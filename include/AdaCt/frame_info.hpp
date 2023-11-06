#pragma once

#include "AdaCt/utility.h"
#include "AdaCt/optPose.hpp"
#include "AdaCt/voxelmap.hpp"
#include <unordered_map>

class frame_info
{
public:
    /* data */
    OptPose pose;
    double headertime, timeStart, timeEnd;
    int frame_id;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_deskew;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_world;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori_downsample;

    pcl::VoxelGrid<PointXYZIRT> downSamplefliter;

    std::vector<double> timeVec;
    // pcl::VoxelGrid<PointType> downSamplefliter;
    double  downsampleLeafsize;
    

public:

    frame_info(){
        downsampleLeafsize=0.4;
        pose.initialMotion();

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
    };
    frame_info(double headertime_,double timeStart_, double timeEnd_, int frame_id_){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;

        pose.initialMotion();

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
    }

    void setFrameTime(double headertime_,double timeStart_, double timeEnd_, int frame_id_ ){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;
    }

    void setOricloud(pcl::PointCloud<PointXYZIRT>::ConstPtr ori_cloud){
        pcl::copyPointCloud(*ori_cloud, *cloud_ori);
        //ROS_INFO("%f", cloud_ori->points[1].timestamp);
        cloud_deskew->clear();
        cloud_world->clear();
        cloud_ori_downsample->clear();
        timeVec.resize(cloud_ori->size());
//        cloud_deskew->resize(cloud_ori->size());
//        cloud_world->resize(cloud_ori->size());
//        cloud_ori_downsample->resize(cloud_ori->size());
    }


    void setMotion(const OptPose currpose){
        pose=currpose;
    }

    void setMotion(const Eigen::Quaterniond begin_quat, const Eigen::Quaterniond end_quat, 
                    const Eigen::Vector3d begin_trans, const Eigen::Vector3d end_trans ){
        pose.begin_pose=Sophus::SE3d(begin_quat, begin_trans);
        pose.end_pose=Sophus::SE3d(end_quat, end_trans);

    }

//    void downSampleOriCloud(){
//        //ROS_INFO("%s",downSamplefliter.getFilterFieldName().c_str());
//        cloud_ori_downsample->clear();
//
//        downSamplefliter.setLeafSize(downsampleLeafsize,downsampleLeafsize,downsampleLeafsize);
//        downSamplefliter.setInputCloud(cloud_ori);
//
//        downSamplefliter.filter(*cloud_ori_downsample);//downsample wouldn't process timestamp（voxel middle ）
////        ROS_INFO("%f",cloud_ori->points[1].x);
//        ROS_INFO("%f",cloud_ori_downsample->points[1].ring);
//        ROS_INFO("%lu, %lu",cloud_ori->size(),cloud_ori_downsample->size());
//
//    }

//    void grid_sample(){
//        tsl::robin_map<Voxel, PointXYZIRT> grid;
//        grid.reserve(size_t(cloud_ori->size()));
//
//
//    }
   
    //use opt_pose to change cloud_ori to cloud_deskew
    void Distortion(){
        cloud_deskew->clear();
        cloud_deskew->resize(cloud_ori->size());
        for(int i=0;i<cloud_ori->size();i++){
            PointXYZIRT temp=cloud_ori->points[i];
            //ROS_INFO("%f, %f,%f", cloud_ori->points[i].timestamp, timeStart, timeEnd);
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);
            //ROS_INFO("alpha:%f",alpha);
            SE3 temp_T_world=pose.linearInplote(alpha);
            SE3 temp_T_end = pose.end_pose.inverse() * temp_T_world;
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_end * temp_P;//to the end

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            cloud_deskew->points[i]=temp;
        }
        return;
    }

    void TranstoWorld(){
        cloud_world->clear();
        cloud_world->resize(cloud_ori->size());
        for(int i=0;i<cloud_ori->size();i++){
            PointXYZIRT temp=cloud_ori->points[i];
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);
            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points[i]=temp;
        }
        return;
    }

    void update(){
        cloud_world->clear();
        cloud_world->resize(cloud_ori->size());
        cloud_deskew->clear();
        cloud_deskew->resize(cloud_ori->size());
        for(int i=0;i<cloud_ori->size();i++){
            PointXYZIRT temp=cloud_ori->points[i];
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);
            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points[i]=temp;

            temp_P = pose.end_pose.inverse() * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            cloud_deskew->points[i]=temp;
        }
        return;
    }

    void Reset(){

        headertime=timeStart=timeEnd=0;

        cloud_ori->clear();
        cloud_deskew->clear();
        cloud_world->clear();
        cloud_ori_downsample->clear();

        pose.initialMotion();

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
    }
    ~frame_info(){};

    void normalize(){
        pose.normalize();
    }
    
    V3D getEndTrans(){
        return pose.endTrans();
    }

    V3D getBeginTrans(){
        return pose.beginTrans();
    }

    Eigen::Quaterniond endQaut(){
        return pose.endQuat();
    }

    Eigen::Quaterniond beginQuat(){
        return pose.beginQuat();
    }

    M3D getEndRot(){
        return pose.endRot();
    }

    M3D getBeginRot(){
        return pose.endRot();
    }
};




