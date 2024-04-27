#pragma once


#include <iostream>
#include <fstream>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <math.h>
#include <omp.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#include "livox_ros_driver/CustomMsg.h"

#include "types.hpp"


#define INDOOR_THRESHOLD 20.0
using namespace std;



enum LidarType{RS, VLP, AVIA, LIVOX, HESAI, OUST64};
enum ENVTYPE{NARROW=1, INDOOR, OUTDOOR, OPEN};
enum ENVFEATURE{STATIC, DYNAMIC};
enum MOTION_GRADE{SMOOTH=1, TURN, AGGRESSIVE, EXTREME};

// struct PointXYZIRT
// {
//     PCL_ADD_POINT4D;                // pcl宏定义，加入点的坐标
//     PCL_ADD_INTENSITY;              // pcl宏定义，加入强度
//     uint16_t ring;                  // ring
//     double timestamp;               // 时间
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符内存对齐
// } EIGEN_ALIGN16;                    // 强制sse填充
// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))


// typedef PointXYZIRT PointType;

class configParam{
public:
    ros::NodeHandle nh;

    bool debug_print;
    std::string lidar_topic;
    int lidar_type;

    std::string odometry_frame;

    fstream store_io;
    std::string store_fold;

    //some hyper-parameters

    double downsampleLeafsize;
    int initframe_num;

    double map_resolution;
    double local_map_size;

    int max_iter_nums;

    double blind;
    double max_range;
    int N_SCAN;
    int Horizon_SCAN;
    int groundScanInd;
    double vertical_fov;
    double ang_res_x;
    double ang_res_y;
    double segmentAlphaX;
    double segmentAlphaY;
    double segmentTheta = 60.0/180.0 *M_PI;
    int segmentValidPointNum = 5;
    int segmentValidLineNum = 3;

    float edgeThreshold = 0.1;
    float surfThreshold = 0.1;
    float nearestFeatureSearchSqDist = 25;

    ENVTYPE envtype;
    ENVTYPE lastenv;
    ENVFEATURE envfeature;
    MOTION_GRADE ego_motion_grade;
    bool keyframe_valid;
    bool seg_frame;

    configParam(){
        nh.param<bool>("AdaCt/debug_print",debug_print,true);
//        nh.param<int>("AdaCt/lidar_type",lidar_type,RS);
//        nh.param<std::string>("AdaCt/lidar_topic",lidar_topic,"/rslidar_points");
        nh.param<int>("AdaCt/lidar_type",lidar_type,OUST64);
        nh.param<std::string>("AdaCt/lidar_topic",lidar_topic,"/os_cloud_node/points");
//        nh.param<int>("AdaCt/lidar_type",lidar_type,HESAI);
//        nh.param<std::string>("AdaCt/lidar_topic",lidar_topic,"/hesai/pandar");
//        nh.param<int>("AdaCt/lidar_type",lidar_type,VLP);
//        nh.param<std::string>("AdaCt/lidar_topic",lidar_topic,"/velodyne_points");
        nh.param<std::string>("", store_fold,"/home/wjj/SLAM_WS/lslam_ws/AdaCt_ws/src/AdaCt/store_fold/");
        nh.param<int>("AdaCt/initframeNum", initframe_num,20);
        nh.param<double>("AdaCT/map_resolution",map_resolution,0.5);
        nh.param<double>("AdaCt/local_map_size",local_map_size,1000);
        nh.param<std::string>("AdaCt/odometry_frame",odometry_frame,"odometry");
        nh.param<int>("",max_iter_nums,5);
        nh.param<int>("",N_SCAN,32);
        nh.param<int>("", Horizon_SCAN, 1800);
        nh.param<double>("",vertical_fov,41.3);
        nh.param<int>("",groundScanInd,20);//< 32
        nh.param<double>("",blind,0.2);
        nh.param<double>("",max_range,120);
        ang_res_x = (double )360/Horizon_SCAN;
        ang_res_y = (double )vertical_fov/(N_SCAN-1);
        segmentAlphaX = ang_res_x / 180.0 * M_PI;
        //segmentAlphaY = ang_res_y / 180.0 * M_PI;
        segmentAlphaY = 41.33/180*M_PI;
        keyframe_valid = false;

    }
};
