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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>


struct PointVelodyne{
    PCL_ADD_POINT4D;                // pcl宏定义，加入点的坐标
    PCL_ADD_INTENSITY;              // pcl宏定义，加入强度
    uint16_t ring;                  // ring
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointVelodyne,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

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


//struct common point type
typedef struct CommonPointType{
    Eigen::Vector3d point;
    Eigen::Vector3d point_world;
    uint16_t ring;
    double timestamp;
    float intensity;
    //others property
    Eigen::Matrix3d cov;
    double alpha;

} PointType;

//plane
typedef struct Plane {
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    Eigen::Vector3d y_normal;
    Eigen::Vector3d x_normal;
    Eigen::Matrix3d covariance;
    Eigen::Matrix<double, 6, 6> plane_cov;
    float radius = 0;
    float min_eigen_value = 1;
    float mid_eigen_value = 1;
    float max_eigen_value = 1;
    float d = 0;//ax+by+cz+d=1 方程中的d
    double A2D;
    int points_size = 0;

    bool is_plane = false;
    bool is_init = false;
    int id;
    // is_update and last_update_points_size are only for publish plane
    bool is_update = false;
    int last_update_points_size = 0;
    bool update_enable = true;
} Plane;

typedef struct ptpl {
    Eigen::Vector3d point;
    Eigen::Vector3d point_world;
    Eigen::Vector3d normal;
    Eigen::Vector3d center;
    Eigen::Matrix<double, 6, 6> plane_cov;
    double d;
    double point_alpha;
    double A2D;
    int layer;
} ptpl;

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