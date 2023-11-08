//
// Created by wjj on 2023/11/8.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>

class Transform{
private:
    tf::TransformBroadcaster camera2init;
    tf::Quaternion q;
    tf::Vector3 t_w_curr;
    tf::Transform trans;

    ros::NodeHandle nh;
    ros::Subscriber subOdom;

public:
    Transform(){
        //camera_init是原点的，camera是当前的，使用匹配后的位姿作为camera到camera_init的变换
        //这样，提取到的边缘点平面点rviz显示会比较好
        subOdom=nh.subscribe<nav_msgs::Odometry>("/adact/odometry",5,&Transform::OdoHandler,this);


    }
    void OdoHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        //ROS_INFO("SUB!");
        t_w_curr.setValue(laserOdometry->pose.pose.position.x,laserOdometry->pose.pose.position.y,
                          laserOdometry->pose.pose.position.z);
        q.setW(laserOdometry->pose.pose.orientation.w);
        q.setX(laserOdometry->pose.pose.orientation.x);
        q.setY(laserOdometry->pose.pose.orientation.y);
        q.setZ(laserOdometry->pose.pose.orientation.z);

        trans.setOrigin(t_w_curr);
        trans.setRotation(q);
        camera2init.sendTransform(tf::StampedTransform(trans,laserOdometry->header.stamp,"map","odometry"));
    }
};


int main(int argc ,char** argv ){
    ros::init(argc ,argv,"transform");
    Transform TFborcast;
    ros::spin();
    return 0;
}