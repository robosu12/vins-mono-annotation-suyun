#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "loop-closure/loop_closure.h"
#include "loop-closure/keyframe.h"
#include "loop-closure/keyframe_database.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_posegraph_buf;
queue<int> optimize_posegraph_buf;
queue<KeyFrame*> keyframe_buf;
queue<RetriveData> retrive_data_buf;

int sum_of_wait = 0;//这个变量没有使用

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

queue<pair<cv::Mat, double>> image_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;

int global_frame_cnt = 0;
//camera param
camodocal::CameraPtr m_camera;
vector<int> erase_index;
std_msgs::Header cur_header;
Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};
Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

/**
 * @brief 对其imu和图像数据进行初步对齐，使得一副图像对应多组imu数据，并确保相邻图像对应时间戳内的所有IMU数据
 *
 */
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true) //取出当前图像buf中的一帧数据，和小于此帧图像时间戳的所有imu数据
    {
        if (imu_buf.empty() || feature_buf.empty()) //如果imu或者图像的buf是空的，直接返回
            return measurements;

        if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp)) // 若imu和图像buf都有数据之后，
        {                                                                        // 则最新的imu时间戳肯定比最早的图像要大
            ROS_WARN("wait for imu, only should happen at the beginning");       // 否则是还没有imu数据
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp)) //如果buf中最早的imu比图像时间戳还要晚，
        {                                                                         //即图像数据之前没有imu数据，则扔掉最早的图像
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front(); //取出buf中最早的一帧图像数据
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;

        //图像数据(img_msg)，对应多组在时间戳内的imu数据,然后塞入measurements
        while (imu_buf.front()->header.stamp <= img_msg->header.stamp) //取出此图像数据之前的所有imu数据
        {
            IMUs.emplace_back(imu_buf.front()); // emplace是 c++11新特性，效率比push高
            imu_buf.pop();
        }

        measurements.emplace_back(IMUs, img_msg); //构造一组imu与图像测量数据
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void raw_image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    //image_pool[img_msg->header.stamp.toNSec()] = img_ptr->image;
    if(LOOP_CLOSURE)
    {
        i_buf.lock();
        image_buf.push(make_pair(img_ptr->image, img_msg->header.stamp.toSec()));
        i_buf.unlock();
    }
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

//发送IMU数据进行预积分
void send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz)); //计算两帧图像间的imu预积分值
}

//thread:loop detection
void process_loop_detection()
{
    if(loop_closure == NULL)
    {
        const char *voc_file = VOC_FILE.c_str();
        TicToc t_load_voc;
        ROS_DEBUG("loop start loop");
        cout << "voc file: " << voc_file << endl;
        loop_closure = new LoopClosure(voc_file, IMAGE_COL, IMAGE_ROW);
        ROS_DEBUG("loop load vocbulary %lf", t_load_voc.toc());
        loop_closure->initCameraModel(CAM_NAMES);
    }

    while(LOOP_CLOSURE)
    {
        KeyFrame* cur_kf = NULL; 
        m_keyframe_buf.lock();
        while(!keyframe_buf.empty())
        {
            if(cur_kf!=NULL)
                delete cur_kf;
            cur_kf = keyframe_buf.front(); //从主线程滑动窗口中创建的关键帧buf中取出最新帧；
            keyframe_buf.pop();            //在闭环检测过程中插入的关键帧，全都抛弃；
        }                                  //也即，关键帧数据库的增长速度跟闭环检测线程的运行频率相关；
        m_keyframe_buf.unlock();
        if (cur_kf != NULL)
        {
            cur_kf->global_index = global_frame_cnt; //给新关键帧打上全局索引
            m_keyframedatabase_resample.lock();
            keyframe_database.add(cur_kf);           //将新关键帧添加到数据库
            m_keyframedatabase_resample.unlock();

            cv::Mat current_image;
            current_image = cur_kf->image;           //取出关键帧图像

            bool loop_succ = false;
            int old_index = -1;
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            TicToc t_brief;
            cur_kf->extractBrief(current_image);     //提取当前关键帧fast特征点,描述子
            //printf("loop extract %d feature using %lf\n", cur_kf->keypoints.size(), t_brief.toc());
            TicToc t_loopdetect;
            //通过词袋模型进行相似度检测，若检测成功，返回数据库中关键帧索引
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            double t_loop = t_loopdetect.toc();
            ROS_DEBUG("t_loopdetect %f ms", t_loop);
            if(loop_succ) //若找到闭环候选帧
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index); //根据索引取出闭环候选关键帧（从数据库中）
                if (old_kf == NULL)
                {
                    ROS_WARN("NO such frame in keyframe_database");
                    ROS_BREAK();
                }
                ROS_DEBUG("loop succ %d with %drd image", global_frame_cnt, old_index);
                assert(old_index!=-1);
                
                Vector3d T_w_i_old, PnP_T_old;
                Matrix3d R_w_i_old, PnP_R_old;

                old_kf->getPose(T_w_i_old, R_w_i_old);      //获取闭环候选帧位姿
                std::vector<cv::Point2f> measurements_old;
                std::vector<cv::Point2f> measurements_old_norm;
                std::vector<cv::Point2f> measurements_cur;
                std::vector<int> features_id_matched;  
                //进行基于描述子的邻域匹配，计算基本矩阵剔除外点，然后通过pnp计算相对位姿，并剔除外点
                cur_kf->findConnectionWithOldFrame(old_kf, measurements_old, measurements_old_norm, PnP_T_old, PnP_R_old, m_camera);
                measurements_cur = cur_kf->measurements_matched; //measurements_matched 为匹配到的当前帧中的点，和候选帧中点一一对应
                features_id_matched = cur_kf->features_id_matched; //features_id_matched 为匹配到的当前帧的点的id
                // send loop info to VINS relocalization
                int loop_fusion = 0;
                //当剔除后的匹配点仍超过阈值时，认为该候选帧是一个可能正确的闭环帧,并将闭环帧push到retrive_data_buf中，供滑动窗口优化使用，计算相对位姿；
                if( (int)measurements_old_norm.size() > MIN_LOOP_NUM && global_frame_cnt - old_index > 35 && old_index > 30)
                {
                    Quaterniond PnP_Q_old(PnP_R_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.R_old = R_w_i_old;
                    retrive_data.relative_pose = false;
                    retrive_data.relocalized = false;
                    retrive_data.measurements = measurements_old_norm;  //候选帧中匹配点的坐标
                    retrive_data.features_ids = features_id_matched;    //当前帧中匹配点的 id
                    retrive_data.loop_pose[0] = PnP_T_old.x();  
                    retrive_data.loop_pose[1] = PnP_T_old.y();
                    retrive_data.loop_pose[2] = PnP_T_old.z();
                    retrive_data.loop_pose[3] = PnP_Q_old.x();
                    retrive_data.loop_pose[4] = PnP_Q_old.y();
                    retrive_data.loop_pose[5] = PnP_Q_old.z();
                    retrive_data.loop_pose[6] = PnP_Q_old.w();
                    m_retrive_data_buf.lock();
                    //将闭环帧push到retrive_data_buf中，供滑动窗口优化使用，计算相对位姿；
                    //也即：闭环检测线程负责检测相似度高的候选关键帧 -- 滑动窗口计算候选帧与闭环帧的相对位姿，
                    retrive_data_buf.push(retrive_data); 
                    m_retrive_data_buf.unlock();
                    cur_kf->detectLoop(old_index);
                    old_kf->is_looped = 1;
                    loop_fusion = 1;

                    m_update_visualization.lock();
                    keyframe_database.addLoop(old_index);
                    CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
                    pubPoseGraph(posegraph_visualization, cur_header);  
                    m_update_visualization.unlock();
                }


                // visualization loop info
                if(0 && loop_fusion)
                {
                    int COL = current_image.cols;
                    //int ROW = current_image.rows;
                    cv::Mat gray_img, loop_match_img;
                    cv::Mat old_img = old_kf->image;
                    cv::hconcat(old_img, current_image, gray_img);
                    cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
                    cv::Mat loop_match_img2;
                    loop_match_img2 = loop_match_img.clone();
                    /*
                    for(int i = 0; i< (int)cur_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::circle(loop_match_img, old_pts[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img, old_pts[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }
                    ostringstream convert;
                    convert << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<".jpg";
                    cv::imwrite( convert.str().c_str(), loop_match_img);
                    */
                    for(int i = 0; i< (int)measurements_cur.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img2, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::circle(loop_match_img2, measurements_old[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img2, measurements_old[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }

                    ostringstream convert2;
                    convert2 << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<"-2.jpg";
                    cv::imwrite( convert2.str().c_str(), loop_match_img2);
                }
                  
            }
            //release memory
            cur_kf->image.release();
            global_frame_cnt++;

            //虽然前面已经仅保留每帧图像的位姿和特征点描述子，已扔掉原始图像，但是当运行几小时后仍将无法实时。
            //因此，我们将根据分布密度对数据库进行下采样，保留那些与周围帧的位置和角度不同的,而丢弃集中在一起的帧
            if (t_loop > 1000 || keyframe_database.size() > MAX_KEYFRAME_NUM)
            {
                m_keyframedatabase_resample.lock();
                erase_index.clear();
                keyframe_database.downsample(erase_index);
                m_keyframedatabase_resample.unlock();
                if(!erase_index.empty())
                    loop_closure->eraseIndex(erase_index);
            }
        }
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

//thread: pose_graph optimization
void process_pose_graph()
{
    while(true)
    {
        m_posegraph_buf.lock();
        int index = -1;
        while (!optimize_posegraph_buf.empty())
        {
            index = optimize_posegraph_buf.front(); //只需要最新检测到的闭环帧；
            optimize_posegraph_buf.pop();
        }
        m_posegraph_buf.unlock();
        if(index != -1)     //只有检测到有效的闭环后，才会进行全局位姿图优化；
        {
            Vector3d correct_t = Vector3d::Zero();
            Matrix3d correct_r = Matrix3d::Identity();
            TicToc t_posegraph;
            keyframe_database.optimize4DoFLoopPoseGraph(index,
                                                    correct_t,
                                                    correct_r);
            ROS_DEBUG("t_posegraph %f ms", t_posegraph.toc());
            m_loop_drift.lock();
            relocalize_r = correct_r;
            relocalize_t = correct_t;
            m_loop_drift.unlock();
            m_update_visualization.lock();
            keyframe_database.updateVisualization();
            CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
            m_update_visualization.unlock();
            pubOdometry(estimator, cur_header, relocalize_t, relocalize_r);
            pubPoseGraph(posegraph_visualization, cur_header); 
            nav_msgs::Path refine_path = keyframe_database.getPath();
            updateLoopPath(refine_path);
        }

        std::chrono::milliseconds dura(5000);
        std::this_thread::sleep_for(dura);
    }
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {      //对imu和图像数据进行初步对齐，取出buf中每幅图像时间内所有imu数据，push到vector中
            return (measurements = getMeasurements()).size() != 0; 
                 });
        lk.unlock();

        for (auto &measurement : measurements)
        {
            //分别取出各段imu数据，进行预积分，并对系统位置Ps，速度Vs，旋转Rs通过IMU测量值进行更新
            for (auto &imu_msg : measurement.first)
                send_imu(imu_msg); //计算两帧图像间的imu预积分值

            //取出对应这段的vision data
            auto img_msg = measurement.second;
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Vector3d>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5; // 取出每个特征点信息
                int feature_id = v / NUM_OF_CAM; // id
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x; //归一化平面坐标
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                ROS_ASSERT(z == 1);
                image[feature_id].emplace_back(camera_id, Vector3d(x, y, z)); //构建图像观测数据
            }
            estimator.processImage(image, img_msg->header); //处理图像数据，图像数据包括跟踪到的特征点和新采集的特征点
            /**
            *** start build keyframe database for loop closure
            **/
            if(LOOP_CLOSURE)
            {
                // remove previous loop
                vector<RetriveData>::iterator it = estimator.retrive_data_vector.begin();
                //闭环候选帧必须是当前滑动窗口中的帧，也就是闭环必须是当前滑动窗口中的帧检测到的
                for(; it != estimator.retrive_data_vector.end(); )
                {
                    if ((*it).header < estimator.Headers[0].stamp.toSec()) //如果比滑动窗口中最老帧的时间还早，直接删除
                    {
                        it = estimator.retrive_data_vector.erase(it);
                    }
                    else
                        it++;
                }
                m_retrive_data_buf.lock();
                while(!retrive_data_buf.empty()) //如果闭环候选帧buf不为空，也即闭环线程检测到了闭环候选帧
                {
                    RetriveData tmp_retrive_data = retrive_data_buf.front(); //从闭环检测线程检测到的闭环候选帧中取出一帧
                    retrive_data_buf.pop();
                    estimator.retrive_data_vector.push_back(tmp_retrive_data); //加入闭环候选帧向量，供滑动窗口优化使用
                }
                m_retrive_data_buf.unlock();
                //WINDOW_SIZE - 2 is key frame , 构造关键帧buf -- 供闭环检测线程使用；
                if(estimator.marginalization_flag == 0 && estimator.solver_flag == estimator.NON_LINEAR)
                {   
                    Vector3d vio_T_w_i = estimator.Ps[WINDOW_SIZE - 2];
                    Matrix3d vio_R_w_i = estimator.Rs[WINDOW_SIZE - 2];
                    i_buf.lock();
                    while(!image_buf.empty() && image_buf.front().second < estimator.Headers[WINDOW_SIZE - 2].stamp.toSec())
                    {
                        image_buf.pop(); //删掉原始图像缓冲区中比滑窗中第8帧时间还要早的数据，保证取到最新的数据进行闭环检测
                    }
                    i_buf.unlock();
                    //assert(estimator.Headers[WINDOW_SIZE - 1].stamp.toSec() == image_buf.front().second);
                    // relative_T   i-1_T_i relative_R  i-1_R_i
                    cv::Mat KeyFrame_image;
                    KeyFrame_image = image_buf.front().first; //从图像buf中取出一帧，构建关键帧进行闭环检测；
                    
                    const char *pattern_file = PATTERN_FILE.c_str();
                    Vector3d cur_T;
                    Matrix3d cur_R;
                    cur_T = relocalize_r * vio_T_w_i + relocalize_t;
                    cur_R = relocalize_r * vio_R_w_i;
                    KeyFrame* keyframe = new KeyFrame(estimator.Headers[WINDOW_SIZE - 2].stamp.toSec(), vio_T_w_i, vio_R_w_i, cur_T, cur_R, image_buf.front().first, pattern_file);
                    keyframe->setExtrinsic(estimator.tic[0], estimator.ric[0]);
                    keyframe->buildKeyFrameFeatures(estimator, m_camera); //取出窗口中第（WINDOW_SIZE - 2）数据，构造关键帧
                    m_keyframe_buf.lock();
                    keyframe_buf.push(keyframe); //将关键帧插入到关键帧buf，供闭环检测线程使用
                    m_keyframe_buf.unlock();
                    // update loop info
                    //如果滑动窗口计算出有效的闭环信息（当前帧与闭环帧的相对位姿），更新闭环帧之间的联系
                    if (!estimator.retrive_data_vector.empty() && estimator.retrive_data_vector[0].relative_pose)
                    {
                        if(estimator.Headers[0].stamp.toSec() == estimator.retrive_data_vector[0].header)
                        {
                            KeyFrame* cur_kf = keyframe_database.getKeyframe(estimator.retrive_data_vector[0].cur_index);                            
                            if (abs(estimator.retrive_data_vector[0].relative_yaw) > 30.0 || estimator.retrive_data_vector[0].relative_t.norm() > 20.0)
                            {
                                ROS_DEBUG("Wrong loop"); //如果闭环帧与当前帧相对视角和位置超过一定范围，则不接受闭环；
                                cur_kf->removeLoop();
                            }
                            else 
                            {    //更新当前关键帧与闭环帧间的连接关系，包括相对平移，旋转，视角差；
                                cur_kf->updateLoopConnection( estimator.retrive_data_vector[0].relative_t, 
                                                              estimator.retrive_data_vector[0].relative_q, 
                                                              estimator.retrive_data_vector[0].relative_yaw);
                                m_posegraph_buf.lock();
                                //将有效闭环帧加入到位姿图优化buf中 -- 供posegraph线程使用；
                                optimize_posegraph_buf.push(estimator.retrive_data_vector[0].cur_index);
                                m_posegraph_buf.unlock();
                            }
                        }
                    }
                }
            }
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            cur_header = header;
            m_loop_drift.lock();
            if (estimator.relocalize)
            {
                relocalize_t = estimator.relocalize_t;
                relocalize_r = estimator.relocalize_r;
            }
            pubOdometry(estimator, header, relocalize_t, relocalize_r);
            pubKeyPoses(estimator, header, relocalize_t, relocalize_r);
            pubCameraPose(estimator, header, relocalize_t, relocalize_r);
            pubPointCloud(estimator, header, relocalize_t, relocalize_r);
            pubTF(estimator, header, relocalize_t, relocalize_r);
            m_loop_drift.unlock();
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    //RViz相关话题
    registerPub(n);

    //订阅IMU,feature,image的topic,然后在各自的回调里处理消息
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);

    std::thread measurement_process{process};
    std::thread loop_detection, pose_graph;
    if (LOOP_CLOSURE)
    {
        ROS_WARN("LOOP_CLOSURE true");
        loop_detection = std::thread(process_loop_detection);   
        pose_graph = std::thread(process_pose_graph);
        m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES);
    }
    ros::spin();

    return 0;
}
