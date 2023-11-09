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

//-----------------------------Voxel-----------------------------------------
struct Voxel{
    Voxel() =default;
//    Voxel(){
//        x=y=z=-1;
//    }

    Voxel(int x_,int y_,int z_){
        x=x_;
        y=y_;
        z=z_;
    }
    int x=-1,y=-1,z=-1;
    inline bool operator ==(const Voxel &other) const{
        return other.x==x && other.y==y && other.z==z;
    }

    inline bool operator != (const Voxel &other) const{
        return !(*this == other);
    }

    inline bool operator <(const Voxel &other) const{
        return x<other.x || (x==other.x && (y<other.y || (y==other.y && z<other.z)));
    }


    static Voxel Coordinates(const Eigen::Vector3d point,double voxel_size){
        Voxel voxel;
        voxel.x=int(point(0)/voxel_size);
        voxel.y=int(point(1)/voxel_size);
        voxel.z=int(point(2)/voxel_size);
        return voxel;
    }
};

namespace std {
    template<>
    struct hash<Voxel> {
        std::size_t operator()(const Voxel &vox) const {
            // const std::hash<int32_t> hasher;
            const size_t kP1 = 73856093;//???
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;

            // return ((hasher(vox.x) ^ (hasher(vox.y) << 1)) >> 1) ^ (hasher(vox.z) << 1) >> 1;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
        }
    };
}

//VoxelBlock is the voxel

template<typename T>
double PointDis(T p1, T p2){
    return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
}

template<class PointT>
struct VoxelBlock{
    explicit VoxelBlock(int num_points=20):num_points_(num_points){
        points.reserve(num_points);
       // timestamps.reserve(num_points);
        mid_point.x=0;
        mid_point.y=0;
        mid_point.z=0;
    }

    std::vector<PointT> points;
    //std::vector<double> timestamps;

    void calculateMid(PointT newP){
        mid_point.x = (mid_point.x * points.size()+newP.x)/(points.size()+1);
        mid_point.y = (mid_point.y * points.size()+newP.y)/(points.size()+1);
        mid_point.z = (mid_point.z * points.size()+newP.z)/(points.size()+1);
    }

    bool isFull() const {return num_points_ == points.size();}
    //bool legal() const {return points.size() == timestamps.size()};
    void addPoint(const PointT &point){
        if(points.size()>=num_points_){
            //ROS_INFO("full!");
            //return;
            //vec : double
            num_points_=num_points_+num_points_;

        }
        //mid_point = (mid_point * points.size() + point)/(points.size()+1);
        calculateMid(point);
        points.push_back(point);
//        if( !legal()) {
//            ROS_ERROR("VOXEL BLOCK INSERT ERROR!");
//        }
        return;
    }

//    void addPoint(const Eigen::Vector3d &point, const double &timestamp){
//        if(point.size()>=num_points_){
//            //ROS_INFO("full!");
//            //return;
//            //vec : double
//            num_points_=num_points_+num_points_;
//
//
//        }
//        mid_point = (mid_point * points.size() + point)/(points.size()+1);
//        points.push_back(point);
//        //timestamps.push_back(timestamp);
//
////        if( !legal()) {
////            ROS_ERROR("VOXEL BLOCK INSERT ERROR!");
////        }
//    }

     PointT findCloseToMid() const {
        double min_dis = std::numeric_limits<double>::max();
        PointT target_min;
        for(int i=0;i<points.size();i++){
            auto point=points[i];
            double dis = PointDis(point ,mid_point);

            if(dis < min_dis){
                target_min = point;
                min_dis = dis;

            }
        }
        return target_min;
    }

    inline int numPoints() const{
        return points.size();
    }

    inline int Capacity(){
        return num_points_;
    }

private:
    int num_points_;
    PointT mid_point;
};

//-------------------------------for neighbor-----------------------------
using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

struct __Comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
        return std::get<0>(left) < std::get<0>(right);//big to small
    }
};

typedef  std::priority_queue<pair_distance_t,std::vector<pair_distance_t>, __Comparator> Neighbors_queue;

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
