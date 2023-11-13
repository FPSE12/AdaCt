#pragma once

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <vector>
#include <queue>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>



//---------------------pointType-----------------------------------------
struct PointXYZIRT
{
    PCL_ADD_POINT4D;                // pcl宏定义，加入点的坐标
    PCL_ADD_INTENSITY;              // pcl宏定义，加入强度
    uint16_t ring;                  // ring
    double timestamp;               // 时间
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符内存对齐
} EIGEN_ALIGN16;                    // 强制sse填充
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))


//---------------------------------------quat to euler--------------------
struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Eigen::Quaterniond q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    q.normalize();

    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

//-------------------
template<typename PointT>
Eigen::Vector3d getPointXYZ(PointT point){
    Eigen::Vector3d T(point.x,point.y,point.z);
    return T;
}