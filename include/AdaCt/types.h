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
struct VoxelBlock{
    explicit VoxelBlock(int num_points=20):num_points_(num_points){
        points.reserve(num_points);
    }

    std::vector<Eigen::Vector3d> points;

    bool isFull() const {return num_points_ == points.size();}
    void addPoint(const Eigen::Vector3d &point){
        CHECK(num_points_ >= points.size()) << "Voxel Is Full";
        points.push_back(point);
    }

    inline int numPoints() const{
        return points.size();
    }

    inline int Capacity(){
        return num_points_;
    }

private:
    int num_points_;
};


using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

struct __Comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
        return std::get<0>(left) < std::get<0>(right);//big to small
    }
};

typedef  std::priority_queue<pair_distance_t,std::vector<pair_distance_t>, __Comparator> Neighbors_queue;

