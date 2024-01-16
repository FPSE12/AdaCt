#pragma once

#include "AdaCt/utility.h"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


typedef Sophus::SE3d SE3;
typedef Sophus::Matrix3d M3D;
typedef Sophus::Vector3d V3D;

using POSE=pair<Eigen::Quaterniond ,Eigen::Vector3d >;

class OptPose{
public:
    SE3 begin_pose, end_pose;
    OptPose(){
        begin_pose.setRotationMatrix(Sophus::Matrix3d::Identity());
        begin_pose.trans(0,0,0);

        end_pose.setRotationMatrix(Sophus::Matrix3d::Identity());
        end_pose.trans(0,0,0);
    }

    OptPose(SE3 begin_pose_, SE3 end_pose_, double header, double start, double end):
            begin_pose(begin_pose_), end_pose(end_pose_){

    }

    void initialMotion(){
        begin_pose.setRotationMatrix(Sophus::Matrix3d::Identity());
        begin_pose.trans(0,0,0);

        end_pose.setRotationMatrix(Sophus::Matrix3d::Identity());
        end_pose.trans(0,0,0);

    }

    SE3 linearInplote(double alpha){
        if(alpha<0.0 || alpha>1.0){
            ROS_ERROR("OptPose_h wrong! alpha is not in 0-1.alpha: %f",alpha);
            return begin_pose;
        }
        SE3 begin_pose_inv=begin_pose.inverse();
        SE3 delta=begin_pose_inv * end_pose;
        Sophus::Vector6d alpha_delta_se3=alpha*delta.log();
        return begin_pose * (SE3::exp(alpha_delta_se3));
//        Eigen::Quaterniond begin_quat=beginQuat();
//        Eigen::Quaterniond end_quat=endQuat();
//        Eigen::Vector3d begin_trans = beginTrans();
//        Eigen::Vector3d end_trans = endTrans();
//
//        Eigen::Quaterniond inter_quat = begin_quat.normalized().slerp(alpha,end_quat.normalized());
//
//        Eigen::Vector3d  inter_trans = (1- alpha) * begin_trans + alpha * end_trans;
//
//        return SE3(inter_quat.normalized(),inter_trans);

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
    OptPose & operator = (const OptPose & pose){
        if(this != & pose){//防止自赋值！very important
            this->begin_pose=pose.begin_pose;
            this->end_pose=pose.end_pose;
        }
        return * this;

    }

    int compareDiff(const OptPose otherpose){
        Eigen::Quaterniond other_qua=otherpose.end_pose.unit_quaternion();
        Eigen::Quaterniond this_qua=this->end_pose.unit_quaternion();
        double delta_thate = acos(this_qua.dot(other_qua)/(this_qua.norm() * other_qua.norm()));

        Eigen::Vector3d other_trans = otherpose.end_pose.translation();
        Eigen::Vector3d this_tran = this->endTrans();
        double delta_trans = (other_trans-this_tran).norm();

        if(delta_thate < 0.05 && delta_trans < 0.01) return 0;
        if(delta_thate > 5 || delta_trans > 0.5) return 2;//see ct-icp's motion threshold

        return 1;
    }

    void normalize(){
        begin_pose.normalize();
        end_pose.normalize();
    }

    M3D endRot(){
        return end_pose.rotationMatrix();
    }
    Eigen::Quaterniond endQuat(){
        return end_pose.unit_quaternion();
    }

    Eigen::Quaterniond beginQuat(){
        return begin_pose.unit_quaternion();
    }
    V3D endTrans(){
        return end_pose.translation();
    }

    M3D beginRot(){
        return begin_pose.rotationMatrix();
    }

    V3D beginTrans(){
        return begin_pose.translation();
    }

};



