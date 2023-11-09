#pragma once 

#include "tsl/robin_map.h"
#include <glog/logging.h>
#include "AdaCt/types.hpp"


#define max_neighbor_nums 5

template<class PointT>
 class Voxelmap{
 public:
     Voxelmap(){
         voxel_size=0.5;
         max_voxel_block_size=20;
         min_distance_between_points=0.1;
         num_points=0;
     }

     Voxel InsertPoint(const PointT &Wpoint){
         Eigen::Vector3d WpointVec(Wpoint.x,Wpoint.y,Wpoint.z);
         Voxel voxel=Voxel::Coordinates(WpointVec,voxel_size);

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
                 double sq_dis = PointDis(_point,Wpoint);
                 if(sq_dis<min_dis){
                     min_dis=sq_dis;
                 }

             }
             if(min_dis>min_distance_between_points * min_distance_between_points){
                 voxelBlock.addPoint(Wpoint);//not used the addpoint function for not defination the num_points in VoxelBlcok
                 return voxel;
             }
         }
     }

     void InsertPointCloud( typename pcl::PointCloud<PointT>::ConstPtr worldCloud){
         for(auto point : worldCloud->points ){
             //Eigen::Vector3d Wpoint(point.x, point.y, point.z);
             InsertPoint(point);
         }
         return;
     }

     void RemoveFarFromLocation(const Eigen::Vector3d &location,double distance){
         std::set<Voxel> voxels_to_erase;
         Eigen::Vector3d voxel_center;
         for(auto &voxel:map){
             voxel_center = Eigen::Vector3d ((voxel.first.x + 0.5)*voxel_size,(voxel.first.y+0.5)*voxel_size, (voxel.first.z+0.5)*voxel_size);
             if((location-voxel_center).norm()>distance){
                 num_points -= voxel.second.points.size();
                 voxels_to_erase.template emplace(voxel.first);
             }
         }

         for(auto &voxel : voxels_to_erase){
             map.erase(voxel);

         }
     }

     bool NeighborSearch_(const PointT & PointW,double searchThreshold, int max_num_beighbor, Neighbors_queue & neighborsQueue){
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


     bool esti_plane(Eigen::Vector4d &pabcd, Neighbors_queue neighborsQueue, double threshold)
     {
         Eigen::Matrix<double, max_neighbor_nums, 3> A;
         Eigen::Matrix<double, max_neighbor_nums, 1> b;
         A.setZero();
         b.setOnes();
         b *= -1.0f;

         std::vector<Eigen::Vector3d> neighbor_vec;
         neighbor_vec.reserve(max_neighbor_nums);
         for (int i = 0; i < max_neighbor_nums; i++)
         {
             neighbor_vec[i](0)=A(i, 0) = std::get<1>(neighborsQueue.top()).x();
             neighbor_vec[i](1)=A(i, 1) = std::get<1>(neighborsQueue.top()).y();
             neighbor_vec[i](2)=A(i, 2) = std::get<1>(neighborsQueue.top()).z();
             neighborsQueue.pop();
         }

         Eigen::Matrix<double, 3, 1> norm_vec = A.colPivHouseholderQr().solve(b);

         double n = norm_vec.norm();

         pabcd(0) = norm_vec(0) / n;
         pabcd(1) = norm_vec(1) / n;
         pabcd(2) = norm_vec(2) / n;
         pabcd(3) = 1.0 / n;

         for (int j = 0; j < max_neighbor_nums; j++)
         {
             if (fabs(pabcd(0) * neighbor_vec[j](0) + pabcd(1) * neighbor_vec[j](1) + pabcd(2) * neighbor_vec[j](2) + pabcd(3)) > threshold)
             {
                 return false;
             }
         }
         return true;
     }

     bool NeighborSearch(const PointT & PointW,double searchThreshold, Neighbors_queue & neighborsQueue, Eigen::Vector4d & pabcd){
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
                             Eigen::Vector3d neighbor(voxel_block.points[i].x,voxel_block.points[i].y,voxel_block.points[i].z);
                             double dis=(PointW_-neighbor).norm();
                             if(dis<searchThreshold){
                                 if(neighborsQueue.size() == max_neighbor_nums){
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

         if(neighborsQueue.size() >= max_neighbor_nums && esti_plane(pabcd, neighborsQueue, 0.1f)){//estiplane?
             double point2plane = PointW_(0) * pabcd(0) + PointW_(1) * pabcd(1) + PointW_(2) * pabcd(2) + pabcd(3);
             double s = 1 - 0.9 * point2plane;
             if(s>0.9){
                 return true;
             }
         }
         return false;
     }

     void clearMap(){
         map.clear();
     }

     int size(){
         return num_points;
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

     tsl::robin_map<Voxel, VoxelBlock<PointT>> map;


    

 };