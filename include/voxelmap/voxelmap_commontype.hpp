#pragma once 

#include "tsl/robin_map.h"
#include <glog/logging.h>
#include <optional>
#include "voxelblock.hpp"
#include "AdaCt/optPose.hpp"

#define max_neighbor_nums 20

#define MAP_VOXEL_SIZE 0.4
#define MIN_DISTANCE_POINT 0.05


 class VoxelmapCommon{
 public:
     VoxelmapCommon(){
         //0.2  ,0.03, 50; 0.5,0.1,40,1.5,0.15,40
         voxel_size=MAP_VOXEL_SIZE;
         max_voxel_block_size=50;
         min_distance_between_points=MIN_DISTANCE_POINT;
         num_points=0;

         frame_count=0;
         max_frames_to_keep=100;
     }

     void updateVoxelMap(std::vector<PointType> & points_vec){
         frame_count++;
         for(auto &point : points_vec){
             Eigen::Vector3d WpointVec = point.point_world;
             Voxel voxel=Voxel::Coordinates(WpointVec,voxel_size);

             if(map.find(voxel)==map.end()){
                map[voxel].emplace_back(point);
                num_points++;
                continue;
             }

             auto &voxelBlock = map[voxel];
             if(voxelBlock.size()<max_voxel_block_size){
                 double min_dis = std::numeric_limits<double>::max();
                 for(auto & mp : voxelBlock){
                     Eigen::Vector3d mp_world = mp.point_world;
                     double sq_dis = (mp_world[0] - WpointVec[0]) * (mp_world[0] - WpointVec[0]) +
                             (mp_world[1] - WpointVec[1]) * (mp_world[1] - WpointVec[1]) +
                             (mp_world[2] - WpointVec[2]) * (mp_world[2] - WpointVec[2]);
                     if(sq_dis<min_dis){
                         min_dis=sq_dis;
                     }

                 }
                 if(min_dis>min_distance_between_points * min_distance_between_points){
                     map[voxel].emplace_back(point);;//not used the addpoint function for not defination the num_points in VoxelBlcok
                     num_points++;
                 }
             }
         }

         return;
     }

     void buildVoxelMap(std::vector<PointType> & points_vec){
         frame_count++;
         for(auto &point : points_vec) {
             Eigen::Vector3d WpointVec = point.point_world;
             Voxel voxel = Voxel::Coordinates(WpointVec, voxel_size);
             map[voxel].emplace_back(point);
         }
     }

     void RemoveFarFromLocation(const Eigen::Vector3d &location,double distance){
         std::set<Voxel> voxels_to_erase;
         Eigen::Vector3d voxel_center;
         for(auto &voxel:map){
             voxel_center = Eigen::Vector3d ((voxel.first.x + 0.5)*voxel_size,(voxel.first.y+0.5)*voxel_size, (voxel.first.z+0.5)*voxel_size);
             if((location-voxel_center).norm()>distance){
                 num_points -= voxel.second.size();
                 voxels_to_erase.template emplace(voxel.first);
             }
         }

         for(auto &voxel : voxels_to_erase){
             map.erase(voxel);

         }
     }

     void getGlobalMap(pcl::PointCloud<PointXYZIRT>::Ptr global_pcl){
         global_pcl->clear();
         for(auto & [_, points] :map){
             for(auto &point :points){
                 PointXYZIRT p;
                 p.x = point.point_world[0];
                 p.y = point.point_world[1];
                 p.z = point.point_world[2];
                 p.intensity = point.intensity;
                 global_pcl->emplace_back(p);
             }
         }
     }

     void clearMap(){
         map.clear();
     }

     int size(){
         return num_points;
     }
     ~VoxelmapCommon()=default;

// private:
     //voxel point
     double voxel_size;
     int max_voxel_block_size;
     double min_distance_between_points;

     double search_max_radius;
     int search_voxel_radius;

     int num_points;

     tsl::robin_map<Voxel, std::vector<PointType>> map;
//     std::unordered_map<Voxel, VoxelBlock<PointT>> map;
     int frame_count;
//     tsl::robin_map<size_t,Frame> frameID_to_frame;
//
     int max_frames_to_keep;
//     std::list<size_t> frame_indices;


 };