#pragma once

#include "utility.h"
#include "voxelmap/voxelmap.hpp"

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
    cloud_ori_downsample->reserve(grid.size());
    for(const auto &[_,points] : grid){
        cloud_ori_downsample->points.push_back(points[0]);
    }
}