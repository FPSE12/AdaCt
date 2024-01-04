#pragma once

#include "AdaCt/utility.h"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


typedef Sophus::SE3d SE3;
typedef Sophus::Matrix3d M3D;
typedef Sophus::Vector3d V3D;

using POSE=pair<Eigen::Quaterniond ,Eigen::Vector3d >;

class OptPoseMultiMode{
public:

    std::vector<SE3> poses;
    int poseNum;
    OptPoseMultiMode(int num=2): poseNum(num){
        poses.resize(poseNum, Sophus::SE3d(Sophus::Matrix3d::Identity(), Sophus::Vector3d(0,0,0)));
    }

    void init(int num){
        poseNum=num;
        poses.resize(poseNum, Sophus::SE3d(Sophus::Matrix3d::Identity(), Sophus::Vector3d(0,0,0)));
    }


    void initialMotion(){
//
        poses.clear();
        poses.resize(poseNum, Sophus::SE3d(Sophus::Matrix3d::Identity(), Sophus::Vector3d(0,0,0)));
    }

    SE3 linearInplote(double alpha){
        if(alpha<0.0 || alpha>1.0){
            ROS_ERROR("OptPose_h wrong! alpha is not in 0-1.alpha: %f",alpha);
            return poses.front();
        }

        int indexB=int(alpha * (poseNum-1));
        alpha = (alpha*(poseNum-1) - indexB);
        SE3 tempB=poses[indexB];
        SE3 tempE=poses[indexB+1];

        SE3 begin_pose_inv=tempB.inverse();
        SE3 delta=begin_pose_inv * tempE;
        Sophus::Vector6d alpha_delta_se3=alpha*delta.log();
        return tempB * (SE3::exp(alpha_delta_se3));


    }

//    POSE linearInplote(double alpha){
//        if(alpha<0.0 || alpha>1.0){
//            ROS_ERROR("OptPose_h wrong! alpha is not in 0-1.alpha: %f",alpha);
//            return pair<Eigen::Quaterniond, Eigen::Vector3d>(begin_pose.unit_quaternion(),begin_pose.translation());
//        }
////        SE3 begin_pose_inv=begin_pose.inverse();
////        SE3 delta=begin_pose_inv * end_pose;
////        Sophus::Vector6d alpha_delta_se3=alpha*delta.log();
////        return begin_pose * (SE3::exp(alpha_delta_se3));
//        Eigen::Quaterniond begin_quat=beginQuat();
//        Eigen::Quaterniond end_quat=endQuat();
//        Eigen::Vector3d begin_trans = beginTrans();
//        Eigen::Vector3d end_trans = endTrans();
//
//        Eigen::Quaterniond inter_quat = begin_quat.normalized().slerp(alpha,end_quat.normalized());
//        inter_quat.normalize();
//
//        Eigen::Vector3d  inter_trans = (1- alpha) * begin_trans + alpha * end_trans;
//
//        return POSE(inter_quat,inter_trans);
//
//    }
    OptPoseMultiMode & operator = (const OptPoseMultiMode & pose){
        poses.resize(pose.poseNum);
        poseNum=pose.poseNum;
        for(int i=0;i<poseNum;i++){
            poses[i]=pose.poses[i];
        }
    }

    SE3 & back(){
        return  poses.back();
    }

    SE3 & front(){
        return poses.front();
    }
    SE3 & operator [](int i){
        return poses[i];
    }

    void clear(){
        poses.clear();
        poseNum=0;
    }

    int size(){
        return poses.size();
    }
    bool compareDiff(const OptPoseMultiMode otherpose){
        Eigen::Quaterniond other_qua=otherpose.poses.back().unit_quaternion();
        Eigen::Quaterniond this_qua=this->poses.back().unit_quaternion();
        double delta_thate = acos(this_qua.dot(other_qua)/(this_qua.norm() * other_qua.norm()));

        Eigen::Vector3d other_trans = otherpose.poses.back().translation();
        Eigen::Vector3d this_tran = this->endTrans();
        double delta_trans = (other_trans-this_tran).norm();

        if(delta_thate < 0.01 && delta_trans < 0.01) return true;

        return false;
    }

    void normalize(){
        for(auto & pose : poses){
            pose.normalize();
        }
    }


    Eigen::Quaterniond endQuat(){
        return poses.back().unit_quaternion();
    }

    Eigen::Quaterniond beginQuat(){
        return poses.front().unit_quaternion();
    }
    V3D endTrans(){
        return poses.back().translation();
    }



    V3D beginTrans(){
        return poses.front().translation();
    }

};



