#pragma once


#include <iostream>
#include <fstream>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "types.h"

using namespace std;

enum lidar_type{RS, VLP, LIVOX};

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

    configParam(){
        nh.param<bool>("AdaCt/debug_print",debug_print,true);
        nh.param<int>("AdaCt/lidar_type",lidar_type,1);
        nh.param<std::string>("AdaCt/lidar_topic",lidar_topic,"/rslidar_points");
        nh.param<int>("AdaCt/initframeNum", initframe_num,20);
        nh.param<double>("AdaCT/map_resolution",map_resolution,0.5);
        nh.param<double>("AdaCt/local_map_size",local_map_size,1000);
        nh.param<std::string>("AdaCt/odometry_frame",odometry_frame,"odometry");

    }
};
