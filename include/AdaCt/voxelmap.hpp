#pragma once 

#include <vector>
#include <queue>

#include <tsl/robin_map.h>




struct Voxel{
public:
    int x=-1,y=-1,z=-1;
    Voxel() =default;
//    Voxel(){
//        x=y=z=-1;
//    }

    Voxel(int x_,int y_,int z_){
        x=x_;
        y=y_;
        z=z_;
    }

    inline bool operator ==(const Voxel &other) const{
        return other.x==x && other.y==y && other.z==z;
    }

    inline bool operator != (const Voxel &other) const{
        return !(*this == other);
    }

    inline bool operator <(const Voxel &other) const{
        return x<other.x || (x==other.x && (y<other.y || (y==other.y && z<other.z)));
    }

    template<typename point3D>
    static Voxel Coordinates(const point3D point,double voxel_size){
        Voxel voxel;
        voxel.x=int(point(0)/voxel_size);
        voxel.y=int(point(1)/voxel_size);
        voxel.z=int(point(2)/voxel_size);
        return voxel;
    }
};

//VoxelBlock is the voxel 
class VoxelBlock{

};

// class Voxelmap{
// public:
//     Voxelmap(){
//         voxel_size=0.5;
//     }

// private:
//     double voxel_size;
//     int max_voxel_block_size;
//     double min_diatance_between_points;

//     double search_max_radius;
//     int search_voxel_radius;

//     tsl::robin_map<Voxel, VoxelBlock> map;

    

// };