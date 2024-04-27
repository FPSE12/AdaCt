#include "AdaCt/utility.h"
#include "AdaCt/frame_info_usecommonPointTypeOnly.hpp"
#include "AdaCt/cloud_convert.hpp"
#include "AdaCt/downSample.hpp"
#include "AdaCt/trajectory.hpp"
//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap_OctoTree.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;

//#define USE_MANNUAL 1

#define MAX_ITER_COUNT 5
#define MAX_ITER_COUNT_SCAN 2

#define MAX_NEIGHBOR_NUM 20

#define A2D_THRESHOLD 0.0 //UNSTRUCTE:low; , structure: >0.4

#define keyframe_threshold_w 5
#define keyframe_threshold_t 0.3

//#define keyframe_valid true
//



class Lidar_odo : public configParam
{
public:
    ros::Subscriber lidar_sub;
    ros::Subscriber lidar_sub_livox;

    ros::Publisher map_pub;
    ros::Publisher odo_pub;
    ros::Publisher traj_pub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_ori_pub;
    ros::Publisher cloud_valid_pub;
    ros::Publisher cloud_world_pub;
    ros::Publisher cloud_downsample_pub;


    // cloud
    std::deque<sensor_msgs::PointCloud2> rosCloudQue;
    std::deque<livox_ros_driver::CustomMsg> rosLivoxCloudQue;

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_valid;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_world;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_deskew;
    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

    // double timeStart, timeEnd, headertime;
//    frame_info curr_frame;

    std::mutex mtx;
    std::mutex mtx_get;

    bool first_flag;
    int frame_count;

    //motion_threshold
    bool isMove = false;
    Sophus::SE3d model_deviation;
    double min_motion_th_ = 0.1;
    double initial_threshold_ = 1.5;
    int num_samples_ = 0;
    double modle_error_sse2 = 0.0;
    double reg_thres  = 1.5;

    // map
    tsl::robin_map<Voxel, OctoTree*> voxel_map;
    float max_voxel_size;
    int max_layer;
    vector<int> layer_size;
    int max_points_size;
    float min_eigen_value;

    //PCA analyse
    Eigen::Vector3f PCALastMainDir;
    Eigen::Vector3f PCALastEigenValue;

    //env judge
    double average_dis = 0;


    //trajectory
    std::vector<OptPose> key_poses;
    OptPose last_pose;
    std::vector<PointType> last_points;

    double trajectory_startTime;
    Trajectory globalTraj;


    int iter_nums; // 5

    //time cost ms
    double init_cost;
    double neighbor_find_average;
    double solve_average;
    double solve_cost;
    double map_cost;
    double all_cost;
    double cost_threshold;
    double cost_robust;

    double a2d_average;

    string store_path;
    fstream store_path_ss;

    CLOUD_CONVERT CC;

    Lidar_odo()
    {
        //ROS_INFO("1");
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_odo::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        //lidar_sub_livox = nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 5, &Lidar_odo::livoxLidarCallback, this, ros::TransportHints().tcpNoDelay());
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/deskew_pointcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);
        cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/world_points",1);
        cloud_downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/downsample",1);


        cloud_valid.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori.reset(new pcl::PointCloud<pcl::PointXYZ>());
        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());

        // Segcloud.reset(new pcl::PointCloud<PointXYZIRT>());

        rosCloudQue.clear();

        //time cost
        init_cost = 0;
        neighbor_find_average = 0;
        solve_average = 0;
        solve_cost = 0;
        map_cost = 0;
        all_cost = 0;
        cost_threshold = 100.0;
        cost_robust = 30.0;


        // params init
        first_flag = true;
        frame_count =-1;
        iter_nums =MAX_ITER_COUNT;

        isMove = false;


        last_pose.initialMotion();
        //mapping params
        max_voxel_size = 0.8;
        max_layer = 2;//0,1,2
        layer_size=vector<int>{10,5,5,5,5};
        max_points_size = 1000;//1000
        min_eigen_value = 0.01;


        last_points.clear();

        store_path = store_fold + "pca_main_dir.txt";
        store_path_ss.open(store_path,ios::out);
        if(!store_path_ss){
            ROS_ERROR("SOTRE_DIR_ERROR!");
        }
        store_path_ss.clear();

    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
        //preprocess();
    }

    void livoxLidarCallback(const livox_ros_driver::CustomMsgConstPtr &laserMsg){
        std::lock_guard<std::mutex> lock(mtx);
        rosLivoxCloudQue.emplace_back(*laserMsg);
    }


    void ENVAnalyse(vector<PointType> & points_vec){
        auto evaluate = std::chrono::steady_clock::now();
        int sample =0;
        average_dis = 0;
        int count=0;
        for(auto & point :points_vec){
            average_dis += point.distance;
            sample++;
        }
        auto  evaluatea_end = std::chrono::steady_clock::now();
        average_dis = average_dis/sample;
        ROS_INFO("DISTANCE: %f", average_dis);//<5 0.2  <10 0.4 >10 ada

        if(average_dis<5.0){
            envtype = ENVTYPE::NARROW;
            keyframe_valid = false;
            seg_frame = false;
            ROS_WARN("NARROW");
        }else if(average_dis < 8.0){
            envtype = ENVTYPE::INDOOR;
            keyframe_valid = false;
            seg_frame = false;
        } else if(average_dis <15.0){
            envtype = ENVTYPE::OUTDOOR;
            keyframe_valid = false;
            seg_frame = true;
        } else{
            envtype = ENVTYPE::OPEN;
            keyframe_valid = false;
            seg_frame = true;
        }
    }

    int PCAAnalyse(double headertime){
        int seg_num=1;
        Eigen::Vector3f PCACurrMainDir;
        Eigen::Vector4f pcaCenter;
        Eigen::Vector3f PCACurrEigenValue;
        pcl::compute3DCentroid(*cloud_ori, pcaCenter);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud_ori, pcaCenter, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver_(covariance,Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorPCA = eigen_solver_.eigenvectors();
        PCACurrMainDir = eigenVectorPCA.col(2);
        PCACurrEigenValue = eigen_solver_.eigenvalues();

        if(!first_flag){
            //angle change
            double value_1 = abs((PCACurrEigenValue[2] / PCALastEigenValue[2])-1);
            double value_2 = abs((PCACurrEigenValue[1] / PCALastEigenValue[1])-1);
            double value_3 = abs((PCACurrEigenValue[0] / PCALastEigenValue[0])-1);
            double angle = acos(PCACurrMainDir.dot(PCALastMainDir)/(PCACurrMainDir.norm()*PCALastMainDir.norm()));
            angle = angle/M_PI * 180;
            if(angle>90.0){
                angle = 180.0-angle;
            }

//            if(angle>2.0){
//                publishCloud(cloud_valid_pub, cloud_ori, headertime, "map");
//            }
            double value = max(max(value_1,value_3),value_2);
//            if(value > 0.15){
//                publishCloud(cloud_valid_pub, cloud_ori, headertime, "map");
//            }


            double all = value>0.15?0:angle;
            if(all > 2.0){
                publishCloud(cloud_valid_pub, cloud_ori, headertime, "map");
            }

            ROS_INFO("angle %f && value: %f && all:%f",angle,value_1,all);

            store_path_ss<<headertime<<" "<<angle<<" "<<value_1<<" "<<value_2<<" "<<value_3<<endl;


        }else{
            first_flag = false;
        }
        PCALastMainDir = PCACurrMainDir;
        PCALastEigenValue = PCACurrEigenValue;
        return seg_num;
    }





    void Process()
    {
        ros::Rate r(100);//100Hz

        while (ros::ok())
        {

            if (rosCloudQue.empty() && rosLivoxCloudQue.empty())
            {
                continue;
                // return;
            }

            auto all_start = std::chrono::steady_clock::now();

            frame_count++;
            ROS_INFO("------------------------------frame_id:%d----------------------------",frame_count);

            //pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori(new pcl::PointCloud<PointXYZIRT>());
            cloud_ori->clear();

            std::vector<PointType> points_vec;
            std::vector<PointType> points_vec_process;


            // get timestamp
            double headertime = 0.0;
            double timeStart = 0.0;
            double timeEnd = 0.0;

            //get cloud
            auto CC_START = std::chrono::steady_clock::now();
            mtx_get.lock();
            if((LidarType)lidar_type == AVIA){
                //ROS_INFO("1");
                livox_ros_driver::CustomMsg cloud_ori_ros = std::move(rosLivoxCloudQue.front());
                rosLivoxCloudQue.pop_front();
                CLOUD_CONVERT cc(cloud_ori_ros,points_vec,headertime,timeStart, timeEnd);
            }else{
                sensor_msgs::PointCloud2 cloud_ori_ros = std::move(rosCloudQue.front());
                //cloud_ori->clear();
                pcl::fromROSMsg(cloud_ori_ros,*cloud_ori);
                rosCloudQue.pop_front();
                //CLOUD_CONVERT cc(cloud_ori_ros,(LidarType)lidar_type, points_vec,headertime,timeStart,timeEnd);
                CC.cloudConvert(cloud_ori_ros,(LidarType)lidar_type, points_vec,headertime,timeStart,timeEnd);
            }
            mtx_get.unlock();
            auto CC_END = std::chrono::steady_clock::now();
            ROS_INFO("cc_COST: %f", std::chrono::duration<double, std::milli>(CC_END - CC_START).count());
//            pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);
//            preprocess(cloud_ori, points_vec,headertime,timeStart, timeEnd);


            auto ENV_START = std::chrono::steady_clock::now();
            keyframe_valid = false;
            ENVAnalyse(points_vec);
            auto ENV_END = std::chrono::steady_clock::now();
            ROS_INFO("env_COST: %f", std::chrono::duration<double, std::milli>(ENV_END - ENV_START).count());

            auto PCA_START = std::chrono::steady_clock::now();
            int seg_num = PCAAnalyse(headertime);
            auto PCA_END = std::chrono::steady_clock::now();
            ROS_INFO("PCA_COST: %f", std::chrono::duration<double, std::milli>(PCA_END - PCA_START).count());

            //publish ori_cloud, world_cloud


            std::vector<PointType>().swap(points_vec);
            std::vector<PointType>().swap(points_vec_process);


            auto all_end = std::chrono::steady_clock::now();
            all_cost = std::chrono::duration<double, std::milli>(all_end-all_start).count();

            publishCloud(cloud_ori_pub, cloud_ori, headertime, "map");

            //release
            r.sleep();
        }
    }


    void publishCloud(ros::Publisher &pub, pcl::PointCloud<PointXYZIRT>::ConstPtr pcl_cloud, double &header_stamp ,string frame){
        if(pub.getNumSubscribers()) {
            ros::Time ros_headertime = ros::Time(header_stamp);
            sensor_msgs::PointCloud2 ros_cloud;
            pcl::toROSMsg(*pcl_cloud, ros_cloud);
            ros_cloud.header.stamp = ros_headertime;
            ros_cloud.header.frame_id = frame;
            pub.publish(ros_cloud);
            return;
        }
    }
    void publishCloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_cloud, double &header_stamp ,string frame){
        if(pub.getNumSubscribers()) {
            ros::Time ros_headertime = ros::Time(header_stamp);
            sensor_msgs::PointCloud2 ros_cloud;
            pcl::toROSMsg(*pcl_cloud, ros_cloud);
            ros_cloud.header.stamp = ros_headertime;
            ros_cloud.header.frame_id = frame;
            pub.publish(ros_cloud);
            return;
        }
    }

    void transVec2Pcl(std::vector<PointType> &points_vec){
        cloud_world->clear();
        cloud_deskew->clear();
        for(auto & point_type : points_vec){
            Eigen::Vector3d world_point = point_type.point_world;
            Eigen::Vector3d deskew_point = point_type.point_deskew;
            Eigen::Vector3d point = point_type.point;
            PointXYZIRT p;
//            p.x = point[0];
//            p.y = point[1];
//            p.z = point[2];
//            p.intensity = point_type.intensity;
//            cloud_ori->points.emplace_back(p);
            p.x = world_point[0];
            p.y = world_point[1];
            p.z = world_point[2];
            cloud_world->points.emplace_back(p);
            p.x = deskew_point[0];
            p.y = deskew_point[1];
            p.z = deskew_point[2];
            cloud_deskew->points.emplace_back(p);
        }


    }

    void publish(std::vector<PointType> & points_vec, double headertime) {
        nav_msgs::Odometry odometry_pose;
        odometry_pose.header.stamp = ros::Time(headertime);
        odometry_pose.header.frame_id = odometry_frame;
        odometry_pose.pose.pose.orientation.w = last_pose.endQuat().w();
        odometry_pose.pose.pose.orientation.x = last_pose.endQuat().x();
        odometry_pose.pose.pose.orientation.y = last_pose.endQuat().y();
        odometry_pose.pose.pose.orientation.z = last_pose.endQuat().z();

        odometry_pose.pose.pose.position.x = last_pose.endTrans().x();
        odometry_pose.pose.pose.position.y = last_pose.endTrans().y();
        odometry_pose.pose.pose.position.z = last_pose.endTrans().z();
        odo_pub.publish(odometry_pose);


        geometry_msgs::PoseStamped geo_odometry_pose;
        geo_odometry_pose.header = odometry_pose.header;
        geo_odometry_pose.pose = odometry_pose.pose.pose;

        globalPath.header.stamp = ros::Time(headertime);
        globalPath.header.frame_id = "map";
        globalPath.poses.emplace_back(geo_odometry_pose);
        traj_pub.publish(globalPath);

        if(cloud_pub.getNumSubscribers()){
            sensor_msgs::PointCloud2 deskew_cloud;
            pcl::toROSMsg(*cloud_deskew,deskew_cloud);
            deskew_cloud.header.stamp = ros::Time(headertime);
            deskew_cloud.header.frame_id= "odometry";
            cloud_pub.publish(deskew_cloud);
        }

        if(cloud_world_pub.getNumSubscribers()){
            sensor_msgs::PointCloud2 world_cloud;
            pcl::toROSMsg(*cloud_world,world_cloud);
            world_cloud.header.stamp = ros::Time(headertime);
            world_cloud.header.frame_id="map";
            cloud_world_pub.publish(world_cloud);
        }

        if(cloud_ori_pub.getNumSubscribers()){
            sensor_msgs::PointCloud2 ori_cloud;
            pcl::toROSMsg(*cloud_ori,ori_cloud);
            ori_cloud.header.stamp = ros::Time(headertime);
            ori_cloud.header.frame_id= "odometry";
            cloud_ori_pub.publish(ori_cloud);
        }












    }


};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "adact_odometry");
    ROS_INFO("START LIDAR PREPROCESS!");

//
    Lidar_odo lp;
//
//    // // 由于process是循环，如果直接运行，不会进入下面的spin，就不会进入点云回调函数，所以要新开一个线程
    std::thread process(&Lidar_odo::Process, &lp);
//
    //process.join();
//    // // spin才会进入回调函数
    ros::spin();
//
//    // // 加入线程
    process.join();
    Eigen::Quaterniond test;
    test.vec();
    return 0;
}