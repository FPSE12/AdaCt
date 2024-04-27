#pragma once

#include "AdaCt/utility.h"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct KNOT{
//    OptPose pose;
    Sophus::SE3d pose;
    double timestamp;
    bool keyPose;

    KNOT(Sophus::SE3d pose_, double timestamp_, bool keypose=false):pose(pose_),timestamp(timestamp_),keyPose(keypose){
    }

    KNOT & operator = (const KNOT & knot_){
        if(this != &knot_){
            this->pose = knot_.pose;
            this->timestamp = knot_.timestamp;
            this->keyPose = knot_.keyPose;
        }
    }

    static KNOT Indentity(double timestamp, bool keypose=false){
        return KNOT(Sophus::SE3d(Sophus::Matrix3d::Identity(),Sophus::Vector3d(0,0,0)), timestamp);
    }
};

class Trajectory{
public:
    std::vector<KNOT> knots;
    int knots_size;
    double max_timestamp;

    Trajectory(){
        knots.clear();
        max_timestamp = 0.0;

        knots_size=0;
    };

    //get pose at timestamp
    Sophus::SE3d getPose(double timestamp){
            if(timestamp > max_timestamp){
                ROS_ERROR("can not query the timestamp:%f > max_timestamp: %f", timestamp, max_timestamp);
                return knots.back().pose;
            }else if(timestamp<0){
                ROS_ERROR("timestamp must > 0!");
                return Sophus::SE3d(Sophus::Matrix3d::Identity(),Sophus::Vector3d(0,0,0));
            }
            for(int i=knots_size-1;i>=0;--i){
                if(knots[i].timestamp < timestamp){
                    double alpha = (timestamp - knots[i].timestamp)/(knots[i+1].timestamp - knots[i].timestamp);
                    Sophus::SE3d delta_pose = knots[i].pose.inverse() * knots[i+1].pose;
                    Sophus::Vector6d delta_log = alpha * delta_pose.log();
                    return knots[i].pose * Sophus::SE3d::exp(delta_log);
                }
            }
            //ROS_ERROR("Return indentity!");
            return Sophus::SE3d(Sophus::Matrix3d::Identity(),Sophus::Vector3d(0,0,0));
    }

    Sophus::SE3d getLastPose(){
        return knots.back().pose;
    }

    //
    void pop_back(){

        knots.pop_back();
        max_timestamp = knots.back().timestamp;
        knots_size--;
    }
    Sophus::SE3d predict(double timestamp){
        if(timestamp < max_timestamp){
            //ROS_ERROR("Preict error! timestamp: %f is in trajectory's range. max_timestamp: %f",timestamp, max_timestamp);
            return getPose(timestamp);
        }
        KNOT pre_knot = knots.back();
        KNOT ppre_knot = knots[knots_size-2];

        double alpha = (timestamp - pre_knot.timestamp)/(pre_knot.timestamp - ppre_knot.timestamp);
        Sophus::SE3d delta_pose = ppre_knot.pose.inverse() * pre_knot.pose;
        return pre_knot.pose * Sophus::SE3d::exp(alpha * delta_pose.log());

    }

    inline void addPose(Sophus::SE3d last_pose, double timestamp){
        if(timestamp < max_timestamp){
           // ROS_WARN("Addpose error! timestamp: %f is in trajectory's range. max_time: %f",timestamp, max_timestamp);
            knots.pop_back();
            knots_size--;
        }else if(timestamp == max_timestamp){
           // ROS_WARN("update last pose");
            knots.pop_back();
            knots_size--;
        }

        max_timestamp = timestamp;
        ++knots_size;
        knots.emplace_back(KNOT(last_pose,timestamp));
    }

    inline void addPose(KNOT last_knot){
        if(last_knot.timestamp < max_timestamp){
            ROS_ERROR("Addpose error! timestamp: %f is in trajectory's range. max_time: %f",last_knot.timestamp, max_timestamp);
            knots.pop_back();
            knots_size--;
        }
        max_timestamp = last_knot.timestamp;
        ++knots_size;
        knots.emplace_back(last_knot);
    }

};