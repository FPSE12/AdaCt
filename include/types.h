#pragma once

#define PCL_NO_PRECOMPILE

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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

