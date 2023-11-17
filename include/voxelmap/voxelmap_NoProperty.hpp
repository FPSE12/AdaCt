#pragma once 

#include "tsl/robin_map.h"
#include <glog/logging.h>
#include <optional>
#include "voxelblock.hpp"
#include "AdaCt/optPose.hpp"

#define max_neighbor_nums 20



template<class PointT>
 class Voxelmap{
 public:
     Voxelmap(){
         //0.2  ,0.03, 50; 0.5,0.1,40,1.5,0.15,40
         voxel_size=0.5;
         max_voxel_block_size=40;
         min_distance_between_points=0.1;
         num_points=0;

         frame_count=0;
         max_frames_to_keep=100;
     }

     //c++17
      std::optional<Voxel> InsertPoint(const PointT &Wpoint, int frame_id=0, int point_id=0){
         Eigen::Vector3d WpointVec(Wpoint.x,Wpoint.y,Wpoint.z);
         Voxel voxel=Voxel::Coordinates(WpointVec,voxel_size);

         if(map.find(voxel)==map.end()){
             map[voxel].points.reserve(max_voxel_block_size);
            // map[voxel].points.push_back(Wpoint);
             //map[voxel].addPoint(Wpoint);
             map[voxel].addPointWithProperties(Wpoint,frame_id,point_id);
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
                 map[voxel].addPointWithProperties(Wpoint,frame_id,point_id);;//not used the addpoint function for not defination the num_points in VoxelBlcok
                 num_points++;
                 return voxel;
             }
         }
         return {} ;
     }

     void InsertPointCloud( typename pcl::PointCloud<PointT>::ConstPtr worldCloud, OptPose curr_pose){

         frame_count++;
         frameID_to_frame[frame_count]={worldCloud, curr_pose};


         std::set<Voxel> voxels_to_update;

         for(int i=0;i<worldCloud->points.size();i++ ){
             //Eigen::Vector3d Wpoint(point.x, point.y, point.z);
             PointT point=worldCloud->points[i];
             auto voxel=InsertPoint(point,frame_count,i);
             voxels_to_update.insert(*voxel);
         }

         for(auto & voxel : voxels_to_update){
                VoxelBlock<PointT> &voxel_block=map[voxel];
                if(voxel_block.points.size()>=voxel_block.MinValidNeighborSize()){
                    voxel_block.computeDescription(NORMAL);
                    voxel_block.normals.clear();
                    for(int i=0;i<voxel_block.points.size();i++){
                        voxel_block.normals.push_back(voxel_block.description.normal);
                        voxel_block.is_normal_computed[i]=true;
                        if(frameID_to_frame.find(voxel_block.frame_ids[i]) != frameID_to_frame.end()){
                            Eigen::Vector3d begin_trans = frameID_to_frame[voxel_block.frame_ids[i]].pose.beginTrans();
                            Eigen::Vector3d point = voxel_block.getPointXYZ(voxel_block.points[i]);
                            if((point-begin_trans).template dot(voxel_block.normals[i])>.0){
                                voxel_block.normals[i] = -voxel_block.normals[i];
                            }
                            voxel_block.is_normal_oriented[i] = true;
                        }else{
                            voxel_block.is_normal_oriented[i] = false;
                        }

                    }

                }

         }

         frame_indices.push_back(frame_count-1);
         while(frame_indices.size()>max_frames_to_keep){
             auto old_idx=frame_indices.front();
             frame_indices.pop_front();
             frameID_to_frame[old_idx].pointCloud = nullptr;
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

     //only consider the point location when trans agressively is unreliable.
     bool NeighborSearch(const PointT & PointW,Eigen::Vector3d sensor_location,double searchThreshold, VoxelBlock<PointT> & neighbor ,Eigen::Vector4d & pabcd){
//            neighbors.reserve(max_num_beighbor);
//            SearchDis.reserve(max_num_beighbor);

         Eigen::Vector3d PointW_(PointW.x, PointW.y,PointW.z);
         Voxel voxel = Voxel::Coordinates(PointW_,voxel_size);
         int kx=voxel.x;
         int ky=voxel.y;
         int kz=voxel.z;

         Neighbors_queue neighborsQueue;

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

                             if(voxel_block.is_normal_oriented[i] && voxel_block.is_normal_computed[i]){
                                double scalar = (sensor_location-PointW_).dot(voxel_block.normals[i]);
                                if(scalar<0.){//theta >90
                                    continue;
                                }
                             }

                             double dis=(PointW_-neighbor).norm();
                             if(dis<searchThreshold){
                                 if(neighborsQueue.size() == max_neighbor_nums){
                                     if(dis < std::get<0>(neighborsQueue.top())){
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

//         if(neighborsQueue.size() >= max_neighbor_nums && esti_plane(pabcd, neighborsQueue, 0.1f)){//estiplane?
////             double point2plane = PointW_(0) * pabcd(0) + PointW_(1) * pabcd(1) + PointW_(2) * pabcd(2) + pabcd(3);
////             double s = 1 - 0.9 * point2plane;
////             if(s>0.9){
//                 while(!neighborsQueue.empty()){
//                     PointT temp;
//                     temp.x=std::get<1>(neighborsQueue.top()).x();
//                     temp.y=std::get<1>(neighborsQueue.top()).y();
//                     temp.z=std::get<1>(neighborsQueue.top()).z();
//                     neighbor.addPoint(temp);
//                     neighborsQueue.pop();
//                 }
////                 neighbor.addPoint(std::get<1>(neighborsQueue.top()));
//
//                 return true;
////             }
//         }
         if(neighborsQueue.size() >= max_neighbor_nums ){//estiplane?

             while(!neighborsQueue.empty()){
                 PointT temp;
                 temp.x=std::get<1>(neighborsQueue.top()).x();
                 temp.y=std::get<1>(neighborsQueue.top()).y();
                 temp.z=std::get<1>(neighborsQueue.top()).z();
                 neighbor.addPoint(temp);
                 neighborsQueue.pop();
             }
//                 neighbor.addPoint(std::get<1>(neighborsQueue.top()));

             return true;
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


     struct Frame{
         typename pcl::PointCloud<PointT>::ConstPtr pointCloud;
         OptPose pose;

     };
     int frame_count;
     tsl::robin_map<size_t,Frame> frameID_to_frame;

     int max_frames_to_keep;
     std::list<size_t> frame_indices;


 };