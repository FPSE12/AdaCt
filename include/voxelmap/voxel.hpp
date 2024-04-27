#pragma once

#include <iostream>
#include <vector>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Geometry>

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


    static Voxel Coordinates(const Eigen::Vector3d &point,double voxel_size){
        Voxel voxel;
        voxel.x=int(point(0)/voxel_size);
        if(voxel.x<0) voxel.x--;
        voxel.y=int(point(1)/voxel_size);
        if(voxel.y<0) voxel.y--;
        voxel.z=int(point(2)/voxel_size);
        if(voxel.z<0) voxel.z--;
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


