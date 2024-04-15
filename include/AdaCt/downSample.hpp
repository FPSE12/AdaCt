#pragma once

#include "utility.h"
#include "voxelmap/voxelmap.hpp"


void subSampleFrame(std::vector<PointType> & frame, double voxel_size){
    tsl::robin_map<Voxel, std::vector<PointType>> grid_map;
    //std::mt19937_64 g;
    //std::shuffle(frame.begin(),frame.end(),g);
    for(auto point : frame){
        Voxel voxel;
        voxel.x = static_cast<short>(point.point[0] / voxel_size);
        if(voxel.x<0) voxel.x--;
        voxel.y = static_cast<short>(point.point[1] / voxel_size);
        if(voxel.y<0) voxel.y--;
        voxel.z = static_cast<short>(point.point[2] / voxel_size);
        if(voxel.z<0) voxel.z--;

            grid_map[voxel].emplace_back(point);


    }
    frame.clear();
    //ROS_INFO("map_size:%d",grid_map.size());
    for( auto & [_, point] : grid_map){
        frame.emplace_back(point[0]);
    }
    //std::shuffle(frame.begin(),frame.end(),g);

}


void subSampleFrame(pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori, pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori_downsample, int voxelSize){
    tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;
    grid.reserve(size_t(cloud_ori->size()));
    Voxel voxel;
//    int blind_voxel=ceil(blind/voxelSize);
    for(int i=0;i<cloud_ori->size();i++){
        voxel.x = static_cast<short>(cloud_ori->points[i].x / voxelSize);
        voxel.y = static_cast<short>(cloud_ori->points[i].y / voxelSize);
        voxel.z = static_cast<short>(cloud_ori->points[i].z / voxelSize);
//        if(voxel.x<blind_voxel && voxel.y<blind_voxel && voxel.z<blind_voxel){
//            continue;
//        }
        grid[voxel].addPoint(cloud_ori->points[i]);
    }
    cloud_ori_downsample->clear();
    //cloud_ori_downsample->reserve(grid.size());
    for(const auto &[_,voxelblock] : grid){
        cloud_ori_downsample->points.emplace_back(voxelblock.points[0]);
    }
}