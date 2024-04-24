#include "AdaCt/utility.h"
#include "AdaCt/frame_info_segment.hpp"

#include "AdaCt/trajectory.hpp"
//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;

#define MAX_ITER_COUNT 5
#define MAX_NEIGHBOR_NUM 20

#define A2D_THRESHOLD 0.4 //UNSTRUCTE:low; , structure: >0.4



//



class Lidar_odo : public configParam
{
public:
    ros::Subscriber lidar_sub;

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
    sensor_msgs::PointCloud2 cloud_ori_ros;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointVelodyne>::Ptr cloud_vel;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;

    pcl::PointCloud<PointXYZIRT>::Ptr cloud_last;


    pcl::PointCloud<PointXYZIRT>::Ptr SegCloud;


    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

    double timeStart, timeEnd, headertime;
    frame_info curr_frame;

    std::mutex mtx;

    bool first_flag;
    int frame_count;

    // debug info
    int beforeRemoveNan, afterRomoveNan;

    // map
    Voxelmap<PointXYZIRT> local_map;
    // opt pose
//    OptPose pre_pose, curr_pose;

    std::vector<OptPose> key_poses;
    OptPose last_pose;
    int iter_nums; // 5

    int motion_evaluate;

    double a2d_average;

    double trajectory_startTime;
    Trajectory globalTraj;


    Lidar_odo()
    {
        //ROS_INFO("1");
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_odo::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/targetcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);
        cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/world_points",1);
        cloud_downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/downsample",1);

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_vel.reset(new pcl::PointCloud<PointVelodyne>());
        cloud_last.reset(new pcl::PointCloud<PointXYZIRT>());

        SegCloud.reset(new pcl::PointCloud<PointXYZIRT>());

        first_flag = true;
        frame_count =-1;

        iter_nums =MAX_ITER_COUNT;


        rosCloudQue.clear();

//        pre_pose.initialMotion();
//        curr_pose.initialMotion();

        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());

//        last_pose.initialMotion();

        motion_evaluate=0;

        trajectory_startTime=0;

    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
        //preprocess();
    }

    void changeCloudFormat(pcl::PointCloud<PointVelodyne>::Ptr vel, pcl::PointCloud<PointXYZIRT>::Ptr rs, double headertime){
        for(auto & point_vel : vel->points){
            PointXYZIRT point_rs;
            point_rs.x = point_vel.x;
            point_rs.y = point_vel.y;
            point_rs.z = point_vel.z;
            point_rs.ring = point_vel.ring;
            point_rs.intensity = point_vel.intensity;
            point_rs.timestamp =(double) point_vel.time + headertime;
//            ROS_INFO("TIMESTAMP: %f",point_rs.timestamp);
            rs->push_back(point_rs);
        }
    }





    void preprocess()
    {
        ros::Rate r(100);//100Hz

        while (ros::ok())
        {

            if (rosCloudQue.size() == 0)
            {
                continue;
                // return;
            }

            auto all_start = std::chrono::steady_clock::now();

            frame_count++;
            ROS_INFO("------------------------------frame_id:%d",frame_count);

            // get pcl cloud
            cloud_ori_ros = std::move(rosCloudQue.front());
            rosCloudQue.pop_front();


            cloud_ori->clear();
            cloud_vel->clear();

            if(lidar_type == VLP){
                //ROS_INFO("VLP");
                pcl::moveFromROSMsg(cloud_ori_ros,*cloud_vel);
                changeCloudFormat(cloud_vel,cloud_ori,headertime);

            }else{
                pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);
            }

            static Eigen::Vector3d last_eigen = Eigen::Vector3d(1,0,0);
            static Eigen::Vector3d last_eigen_1 = Eigen::Vector3d(1,0,0);
            static Eigen::Vector3d last_eigen_2 = Eigen::Vector3d(1,0,0);
            auto test_start = std::chrono::steady_clock::now();
            Eigen::Vector4d pcaCenter;
            pcl::compute3DCentroid(*cloud_ori, pcaCenter);
            Eigen::Matrix3d covariance;
            pcl::computeCovarianceMatrixNormalized(*cloud_ori, pcaCenter, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_(covariance,Eigen::ComputeEigenvectors);
            Eigen::Matrix3d eigenVectorPCA = eigen_solver_.eigenvectors();
            if(first_flag){
                last_eigen = eigenVectorPCA.col(2);//最大的
                last_eigen_1 = eigenVectorPCA.col(1);
                last_eigen_2 = eigenVectorPCA.col(0);
                first_flag = false;
            }else{
                Eigen::Vector3d curr_eigen = eigenVectorPCA.col(2);
                Eigen::Vector3d curr_eigen_1 = eigenVectorPCA.col(1);
                Eigen::Vector3d curr_eigen_2 = eigenVectorPCA.col(0);
                double angle = acos(curr_eigen.dot(last_eigen)/(curr_eigen.norm()*last_eigen.norm()));
                angle = angle/M_PI * 180;
                if(angle>90.0){
                    angle = 180.0-angle;
                }
                double angle_1 = acos(curr_eigen_1.dot(last_eigen_1)/(curr_eigen_1.norm()*last_eigen_1.norm()));
                angle_1 = angle_1/M_PI *180;
                if(angle_1>90.0){
                    angle_1 = 180.0 - angle_1;
                }

                double angle_2= acos(curr_eigen_2.dot(last_eigen_2)/(curr_eigen_2.norm()*last_eigen_2.norm()));
                angle_2 = angle_2/M_PI *180;
                if(angle_2 > 90.0){
                    angle_2 = 180.0 -angle_2;
                }

                last_eigen = curr_eigen;//>2 >5
                last_eigen_1 = curr_eigen_1;
                last_eigen_2 = curr_eigen_2;
                ROS_INFO("ANGLE_max: %f, all:%f", angle, angle+angle_1+angle_2 );
                if(angle>1){//1:2; >5:3  20>4  //all : 1:2, 5:3 15:4
                    publishCloud(cloud_ori_pub, cloud_ori, cloud_ori_ros.header.stamp,"map");
                }
            }
            auto test_end = std::chrono::steady_clock::now();
            //ROS_INFO("PCA:COST :%f", std::chrono::duration<double, std::milli>(test_end - test_start).count());
//           ---------------------------------------------------init------------------------------------

            //publishCloud(cloud_ori_pub, cloud_ori, cloud_ori_ros.header.stamp,"map");


            auto all_end = std::chrono::steady_clock::now();

            double all_cost = std::chrono::duration<double, std::milli >(all_end- all_start).count();

           // ROS_INFO("ALL COST: %f ms", all_cost);

            r.sleep();
        }
    }


    void publishCloud(ros::Publisher &pub, pcl::PointCloud<PointXYZIRT>::ConstPtr pcl_cloud, ros::Time header_stamp ,string frame){
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_cloud, ros_cloud);
        ros_cloud.header.stamp = header_stamp;
        ros_cloud.header.frame_id = frame;
        pub.publish(ros_cloud);
        return ;
    }

    void publish(){
        nav_msgs::Odometry odometry_pose;
        odometry_pose.header.stamp = ros::Time(headertime);
        odometry_pose.header.frame_id = odometry_frame;
        odometry_pose.pose.pose.orientation.w = curr_frame.endQuat().w();
        odometry_pose.pose.pose.orientation.x = curr_frame.endQuat().x();
        odometry_pose.pose.pose.orientation.y = curr_frame.endQuat().y();
        odometry_pose.pose.pose.orientation.z = curr_frame.endQuat().z();

        odometry_pose.pose.pose.position.x = curr_frame.getEndTrans().x();
        odometry_pose.pose.pose.position.y = curr_frame.getEndTrans().y();
        odometry_pose.pose.pose.position.z = curr_frame.getEndTrans().z();


        sensor_msgs::PointCloud2 globalMapRos;
        pcl::toROSMsg(*globalMap, globalMapRos);
        globalMapRos.header.stamp = ros::Time(headertime);
        globalMapRos.header.frame_id = "map";

        geometry_msgs::PoseStamped geo_odometry_pose;
        geo_odometry_pose.header = odometry_pose.header;
        geo_odometry_pose.pose = odometry_pose.pose.pose;

        globalPath.header.stamp = ros::Time(headertime);
        globalPath.header.frame_id = "map";
        globalPath.poses.push_back(geo_odometry_pose);



        sensor_msgs::PointCloud2 world_cloud;
        pcl::toROSMsg(*curr_frame.cloud_world,world_cloud);
        world_cloud.header.stamp = ros::Time(headertime);
        world_cloud.header.frame_id="map";


        sensor_msgs::PointCloud2 ori_cloud;
        pcl::toROSMsg(*curr_frame.cloud_ori,ori_cloud);
        ori_cloud.header.stamp = ros::Time(headertime);
        ori_cloud.header.frame_id= "odometry";

        cloud_ori_pub.publish(ori_cloud);

        cloud_world_pub.publish(world_cloud);

        traj_pub.publish(globalPath);

        odo_pub.publish(odometry_pose);



        map_pub.publish(globalMapRos);


    }

    void debugPrint()
    {
        if (debug_print)
        {
            ROS_INFO("Before remove Nan, size: %d", cloud_ori->size());
            ROS_INFO("After remove Nan, size: %d", cloud_ori->size());
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
    std::thread process(&Lidar_odo::preprocess, &lp);
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