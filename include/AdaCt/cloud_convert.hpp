#pragma once

#include "AdaCt/utility.h"
#include "AdaCt/downSample.hpp"

class CLOUD_CONVERT:public configParam{
public:

    CLOUD_CONVERT(){}

    //构造函数
    CLOUD_CONVERT(sensor_msgs::PointCloud2 & rosMsg, enum LidarType lidarType,
                  std::vector<PointType> & points_vec, double & headertime, double & timeStart, double & timeEnd){
        cloudConvert(rosMsg, lidarType, points_vec, headertime, timeStart,timeEnd);
    }

    CLOUD_CONVERT(livox_ros_driver::CustomMsg & rosMsg,
                  std::vector<PointType> & points_vec, double & headertime, double & timeStart, double & timeEnd){
        cloudConvert(rosMsg, points_vec, headertime, timeStart,timeEnd);
    }

    //handler
    void RoboSenseHandler(sensor_msgs::PointCloud2 & rosMsg,std::vector<PointType> & points_vec,
                          double & headertime, double & timeStart, double & timeEnd){

        headertime = rosMsg.header.stamp.toSec();
        pcl::PointCloud<PointXYZIRT> pcl_cloud;
        pcl::moveFromROSMsg(rosMsg, pcl_cloud);

        for(auto & point_pcl: pcl_cloud.points){
            if(!std::isfinite(point_pcl.x)|| !std::isfinite(point_pcl.y) || !std::isfinite(point_pcl.z) ){
                continue;
            }
            double dis =point_pcl.x * point_pcl.x + point_pcl.y * point_pcl.y + point_pcl.z * point_pcl.z;
            if(dis<blind * blind){
                continue;
            }

            PointType p;
            p.point<<point_pcl.x, point_pcl.y, point_pcl.z;
            p.timestamp = point_pcl.timestamp;
            p.intensity = point_pcl.intensity;
            p.distance = dis;
            points_vec.emplace_back(p);
        }
        //do not calculate the alpha!
        sort(points_vec.begin(), points_vec.end(), timelistvec);
        timeStart = points_vec[0].timestamp;
        timeEnd = points_vec.back().timestamp;

        subSampleFrame(points_vec,0.05);
    }

    void VelodyneHandler(sensor_msgs::PointCloud2 & rosMsg,std::vector<PointType> & points_vec,
                         double & headertime, double & timeStart, double & timeEnd){

        headertime = rosMsg.header.stamp.toSec();
        pcl::PointCloud<PointVelodyne> pcl_cloud;
        pcl::moveFromROSMsg(rosMsg, pcl_cloud);

        for(auto & point_pcl: pcl_cloud.points){
            if(!std::isfinite(point_pcl.x)|| !std::isfinite(point_pcl.y) || !std::isfinite(point_pcl.z) ){
                continue;
            }
            double dis =point_pcl.x * point_pcl.x + point_pcl.y * point_pcl.y + point_pcl.z * point_pcl.z;
            if(dis<blind * blind){
                continue;
            }

            PointType p;
            p.point<<point_pcl.x, point_pcl.y, point_pcl.z;
            p.timestamp = (double)point_pcl.time + headertime;
            p.intensity = point_pcl.intensity;
            p.distance = dis;
            points_vec.emplace_back(p);
        }
        //do not calculate the alpha!
        sort(points_vec.begin(), points_vec.end(), timelistvec);
        timeStart = points_vec[0].timestamp;
        timeEnd = points_vec.back().timestamp;

        subSampleFrame(points_vec,0.05);
    }

    void LivoxHandler(sensor_msgs::PointCloud2 & rosMsg,std::vector<PointType> & points_vec,
                      double & headertime, double & timeStart, double & timeEnd){

    }

    void AviaHandler(livox_ros_driver::CustomMsg & custMsg,std::vector<PointType> & points_vec,
                     double & headertime, double & timeStart, double & timeEnd){
        headertime = custMsg.header.stamp.toSec();
        double base_time = custMsg.timebase;
        for(auto & p : custMsg.points){
            if(!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z)){
                continue;
            }

            double dis = p.x * p.x + p.y *p.y +p.z * p.z;
            if(dis< blind * blind){
                continue;
            }

            PointType pt;
            pt.point<< p.x,p.y,p.z;
            pt.intensity = p.reflectivity;
            pt.timestamp = p.offset_time/(double) 1e9 + headertime;
//            ROS_INFO("time test, offset: %f", p.offset_time);
//            ROS_INFO("base: %f",base_time );
//            ROS_INFO("timestamp: %f", pt.timestamp);
            pt.distance = dis;
            points_vec.emplace_back(pt);
        }
        sort(points_vec.begin(),points_vec.end(), timelistvec);
        timeStart = points_vec[0].timestamp;
        timeEnd = points_vec.back().timestamp;
        subSampleFrame(points_vec,0.01);
    }


    //for avia
    void cloudConvert(livox_ros_driver::CustomMsg & rosMsg,std::vector<PointType> & points_vec,
                      double & headertime, double & timeStart, double & timeEnd){
        AviaHandler(rosMsg, points_vec, headertime, timeStart, timeEnd);
    }

    void cloudConvert(sensor_msgs::PointCloud2 & rosMsg, enum LidarType lidarType,
                      std::vector<PointType> & points_vec, double & headertime, double & timeStart, double & timeEnd){
        switch (lidarType) {
            case LidarType::VLP:
                VelodyneHandler(rosMsg,points_vec,headertime,timeStart,timeEnd);
                break;
            case LidarType::RS:
                RoboSenseHandler(rosMsg,points_vec,headertime,timeStart,timeEnd);
                break;
            case LidarType::LIVOX:
                LivoxHandler(rosMsg,points_vec,headertime,timeStart,timeEnd);
                break;
            default:
                ROS_ERROR("Wrong Lidar Type");
                break;
        }
    }
private:

};
