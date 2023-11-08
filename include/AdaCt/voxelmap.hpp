#pragma once 

#include "tsl/robin_map.h"
#include <glog/logging.h>
#include "AdaCt/types.h"




template<class PointT>
 class Voxelmap{
 public:
     Voxelmap(){
         voxel_size=0.5;
         max_voxel_block_size=20;

         num_points=0;
     }

     Voxel InsertPoint(const Eigen::Vector3d &Wpoint){
         Voxel voxel=Voxel::Coordinates(Wpoint,voxel_size);

         if(map.find(voxel)==map.end()){
             map[voxel].points.reserve(max_voxel_block_size);
             map[voxel].points.push_back(Wpoint);
             num_points++;
             return voxel;

         }

         auto &voxelBlock = map[voxel];

         if(voxelBlock.numPoints()<max_voxel_block_size){
             double min_dis = std::numeric_limits<double>::max();
             for(int i=0;i<voxelBlock.numPoints();i++){
                 auto & _point=voxelBlock.points[i];
                 double sq_dis = (_point-Wpoint).norm();
                 if(sq_dis<min_dis){
                     min_dis=sq_dis;
                 }

             }
             if(min_dis<min_distance_between_points){
                 voxelBlock.points.push_back(Wpoint);//not used the addpoint function for not defination the num_points in VoxelBlcok
                 return voxel;
             }
         }
     }

     void InsertPointCloud(const pcl::PointCloud<PointT> * worldCloud, std::vector<size_t>& out_indices){
         for(auto point : worldCloud->points ){
             Eigen::Vector3d Wpoint(point.x, point.y, point.z);
             InsertPoint(Wpoint);
         }
         return;
     }


     bool NeighborSearch(const PointT & PointW,double searchThreshold, int max_num_beighbor, Neighbors_queue & neighborsQueue){
//            neighbors.reserve(max_num_beighbor);
//            SearchDis.reserve(max_num_beighbor);

            Eigen::Vector3d PointW_(PointW.x, PointW.y,PointW.z);
            Voxel voxel = Voxel::Coordinates(PointW_,voxel_size);
            int kx=voxel.x;
            int ky=voxel.y;
            int kz=voxel.z;

            //Neighbors_queue neighborsQueue;
            for(int kxx=kx-1;kxx<kx+1+1;kxx++){
                for(int kyy=ky-1;kyy<ky+1+1;kyy++){
                    for(int kzz=kz-1;kzz<kz+1+1;kzz++){
                        voxel.x=kxx;
                        voxel.y=kyy;
                        voxel.z=kzz;

                        auto search = map.find(voxel);
                        if(search != map.end()){
                            auto voxel_block = search.value();
                            for(int i=0;i<voxel_block.points.size();i++){
                                Eigen::Vector3d neighbor=voxel_block.points[i];
                                double dis=(PointW_-neighbor).norm();
                                if(dis<searchThreshold){
                                    if(neighborsQueue.size()==max_num_beighbor){
                                        if(dis<std::get<0>(neighborsQueue.top())){
                                            neighborsQueue.pop();
                                            neighborsQueue.emplace(dis,neighbor,voxel);
                                        }
                                    }else{
                                        neighborsQueue.emplace(dis,neighbor,voxel);
                                    }
                                }
                            }
                        }

                    }
                }
            }

            if(neighborsQueue.size()<max_num_beighbor){//estiplane?
                return false;
            }
            return true;
     }


     ~Voxelmap()=default;

 private:
     //voxel point
     double voxel_size;
     int max_voxel_block_size;
     double min_distance_between_points;

     double search_max_radius;
     int search_voxel_radius;

     int num_points;

     tsl::robin_map<Voxel, VoxelBlock> map;


    

 };