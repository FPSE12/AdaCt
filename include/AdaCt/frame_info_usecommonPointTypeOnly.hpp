#pragma once

#include "AdaCt/utility.h"
#include "AdaCt/optPose.hpp"
#include "AdaCt/downSample.hpp"
#include "voxelmap/voxelmap_OctoTree.hpp"
#include <opencv/cv.h>

#define DOWNSAMPLE_VOXEL_SIZE 0.2
#define DOWNSAMPLE_EDGE_VOXEL_SIZE 0.2
#define DOWNSAMPLE_PLANE_VOXEL_SIZE 0.4



const bool pointcmp(PointXYZIRT &x, PointXYZIRT &y){return x.x * x.x + x.y*x.y+x.z*x.z < y.x*y.x+y.y*y.y+y.z*y.z;}
const bool timelist(PointXYZIRT &x, PointXYZIRT &y){return x.timestamp < y.timestamp;}
const bool timelistvec(PointType &x, PointType & y){return x.timestamp < y.timestamp;}
class frame_info
{
public:
    /* data */
    OptPose pose;
    double headertime, timeStart, timeEnd;
    int frame_id;

    //use for calculate
    std::vector<PointType> points;
    std::vector<PointType> all_points;


    std::vector<double> timeVec;
    // pcl::VoxelGrid<PointType> downSamplefliter;
    double voxelSize;

    double blind;

    cv::Mat rangeMat;
    vector<vector<PointXYZIRT *>> point_image;
    int N_SCAN=32;
    int Horizon_SCAN=1800;
    double ang_res_x = (double )360/Horizon_SCAN;

    vector<Eigen::Vector3d> first_line;
    vector<Eigen::Vector3d> last_line;
    vector<pair<double,double>> timestamp_line;



//    std::vector<std::pair<double,double>> distance_voxel_size = {
//            {0.2,  0.2},
//            {5.,  0.4},
//            {15.,  0.8},
//            {100., 1.6},
//            {200., -1}
//    };

public:

    frame_info(){

        voxelSize = DOWNSAMPLE_VOXEL_SIZE;
        blind=0.2;

        pose.initialMotion();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        point_image.resize(N_SCAN,vector<PointXYZIRT *>(Horizon_SCAN, nullptr));

    };
    frame_info(double headertime_,double timeStart_, double timeEnd_, int frame_id_){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;

        pose.initialMotion();


    }

    void setFrameTime(double headertime_,double timeStart_, double timeEnd_, int frame_id_ ){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;
    }



    void setPoints(std::vector<PointType> &ori_cloud){
        all_points.clear();
        ori_cloud.swap(all_points);
        for(auto & point : all_points){
            if(point.timestamp < timeStart || point.timestamp >timeEnd){
                ROS_INFO("ERROR timestamp!");
            }
            point.alpha = (point.timestamp - timeStart)/(timeEnd - timeStart);
        }
    }


    void setMotion(const OptPose currpose){
        pose=currpose;
    }

    void setMotion(const Eigen::Quaterniond begin_quat, const Eigen::Quaterniond end_quat, 
                    const Eigen::Vector3d begin_trans, const Eigen::Vector3d end_trans ){
        pose.begin_pose=Sophus::SE3d(begin_quat, begin_trans);
        pose.end_pose=Sophus::SE3d(end_quat, end_trans);

    }



    void grid_sample_mid_in_vec(double downsample_size){
        points.clear();
        std::vector<PointType> frame_sub;
//        frame_sub.resize(frame.size());
        for(auto & point : all_points){
            frame_sub.emplace_back(point);
        }
        subSampleFrame(frame_sub, downsample_size);
        points.swap(frame_sub);
    }

    void Adaptive_sample_mid_in_vec_NARROW(int average_dis){
        //average_dis = 4;
        std::vector<std::pair<double,double>> distance_voxel_size = {
                {0.2,  0.1},
                {average_dis*1.5, 0.2},
                {200, -1.}
        };


        std::vector<tsl::robin_map<Voxel, std::vector<PointType>>> indices_map(distance_voxel_size.size());
        points.clear();

        for(auto &point : all_points){//do not use the & ,may bechange in point
//            double dis = std::sqrt(point.point[0] * point.point[0] + point.point[1] * point.point[1] +
//                    point.point[2] * point.point[2]);
            double dis = point.distance;
//            ROS_INFO("DIS: %f",dis);
            if(dis >= distance_voxel_size.front().first && dis < distance_voxel_size.back().first) {

                auto lw = std::lower_bound(distance_voxel_size.begin(), distance_voxel_size.end(), dis,
                                           [](const std::pair<double, double> &rhs, double lhs) {
                                               return rhs.first < lhs;
                                           });//first bigger

                auto index_ = std::distance(distance_voxel_size.begin(), lw) - 1;
                double voxel_size = distance_voxel_size[index_].second;

                Voxel voxel;
                voxel.x = static_cast<short>(point.point[0] / voxel_size);
                if(voxel.x<0) voxel.x--;
                voxel.y = static_cast<short>(point.point[1] / voxel_size);
                if(voxel.y<0) voxel.y--;
                voxel.z = static_cast<short>(point.point[2] / voxel_size);
                if(voxel.z<0) voxel.z--;

                indices_map[index_][voxel].emplace_back(point);
            }

        }
        for(const auto &grid_map : indices_map) {
            for(const auto & [_, points_vec] : grid_map){
                PointType midP;

                double min_dis = std::numeric_limits<double>::max();
                for(auto & point : points_vec){
                    midP.point += point.point;
                }
                midP.point /=(double) points_vec.size();

                PointType targetPoint;
                for(auto &point : points_vec){
                    double dis = (midP.point[0]-point.point[0])*(midP.point[0]-point.point[0])+
                                 (midP.point[1]-point.point[1])*(midP.point[1]-point.point[1])+
                                 (midP.point[2]-point.point[2])*(midP.point[2]-point.point[2]);
                    if(dis < min_dis){
                        min_dis = dis;
                        targetPoint = point;
                    }
                }

                points.emplace_back(targetPoint);
            }

        }


    }

    void Adaptive_sample_mid_in_vec_INDOOR(int average_dis){
        //average_dis = 4;
        std::vector<std::pair<double,double>> distance_voxel_size = {
                {0.2,  0.1},
                {2.0,  0.2},
                {average_dis*1.5, 0.4},
                {200, -1.}
        };


        std::vector<tsl::robin_map<Voxel, std::vector<PointType>>> indices_map(distance_voxel_size.size());
        points.clear();

        for(auto &point : all_points){//do not use the & ,may bechange in point
//            double dis = std::sqrt(point.point[0] * point.point[0] + point.point[1] * point.point[1] +
//                    point.point[2] * point.point[2]);
            double dis = point.distance;
//            ROS_INFO("DIS: %f",dis);
            if(dis >= distance_voxel_size.front().first && dis < distance_voxel_size.back().first) {

                auto lw = std::lower_bound(distance_voxel_size.begin(), distance_voxel_size.end(), dis,
                                           [](const std::pair<double, double> &rhs, double lhs) {
                                               return rhs.first < lhs;
                                           });//first bigger

                auto index_ = std::distance(distance_voxel_size.begin(), lw) - 1;
                double voxel_size = distance_voxel_size[index_].second;

                Voxel voxel;
                voxel.x = static_cast<short>(point.point[0] / voxel_size);
                if(voxel.x<0) voxel.x--;
                voxel.y = static_cast<short>(point.point[1] / voxel_size);
                if(voxel.y<0) voxel.y--;
                voxel.z = static_cast<short>(point.point[2] / voxel_size);
                if(voxel.z<0) voxel.z--;

                indices_map[index_][voxel].emplace_back(point);
            }

        }
        for(const auto &grid_map : indices_map) {
            for(const auto & [_, points_vec] : grid_map){
                PointType midP;

                double min_dis = std::numeric_limits<double>::max();
                for(auto & point : points_vec){
                    midP.point += point.point;
                }
                midP.point /=(double) points_vec.size();

                PointType targetPoint;
                for(auto &point : points_vec){
                    double dis = (midP.point[0]-point.point[0])*(midP.point[0]-point.point[0])+
                                 (midP.point[1]-point.point[1])*(midP.point[1]-point.point[1])+
                                 (midP.point[2]-point.point[2])*(midP.point[2]-point.point[2]);
                    if(dis < min_dis){
                        min_dis = dis;
                        targetPoint = point;
                    }
                }

                points.emplace_back(targetPoint);
            }

        }


    }

    void Adaptive_sample_mid_in_vec(int average_dis){
        //average_dis = 4;
        std::vector<std::pair<double,double>> distance_voxel_size = {
                {0.2,  0.1},
                {2.0,  0.2},
                {average_dis/2,   0.4},
                {average_dis,   0.8},
                {2*average_dis,  1.6},
                {min(200,2*average_dis), -1.}
        };


        std::vector<tsl::robin_map<Voxel, std::vector<PointType>>> indices_map(distance_voxel_size.size());
        points.clear();

        for(auto &point : all_points){//do not use the & ,may bechange in point
//            double dis = std::sqrt(point.point[0] * point.point[0] + point.point[1] * point.point[1] +
//                    point.point[2] * point.point[2]);
            double dis = point.distance;
//            ROS_INFO("DIS: %f",dis);
            if(dis >= distance_voxel_size.front().first && dis < distance_voxel_size.back().first) {

                auto lw = std::lower_bound(distance_voxel_size.begin(), distance_voxel_size.end(), dis,
                                           [](const std::pair<double, double> &rhs, double lhs) {
                                               return rhs.first < lhs;
                                           });//first bigger

                auto index_ = std::distance(distance_voxel_size.begin(), lw) - 1;
                double voxel_size = distance_voxel_size[index_].second;

                Voxel voxel;
                voxel.x = static_cast<short>(point.point[0] / voxel_size);
                if(voxel.x<0) voxel.x--;
                voxel.y = static_cast<short>(point.point[1] / voxel_size);
                if(voxel.y<0) voxel.y--;
                voxel.z = static_cast<short>(point.point[2] / voxel_size);
                if(voxel.z<0) voxel.z--;

                indices_map[index_][voxel].emplace_back(point);
            }

        }
        for(const auto &grid_map : indices_map) {
            for(const auto & [_, points_vec] : grid_map){
                PointType midP;

                double min_dis = std::numeric_limits<double>::max();
                for(auto & point : points_vec){
                    midP.point += point.point;
                }
                midP.point /=(double) points_vec.size();

                PointType targetPoint;
                for(auto &point : points_vec){
                    double dis = (midP.point[0]-point.point[0])*(midP.point[0]-point.point[0])+
                            (midP.point[1]-point.point[1])*(midP.point[1]-point.point[1])+
                            (midP.point[2]-point.point[2])*(midP.point[2]-point.point[2]);
                    if(dis < min_dis){
                        min_dis = dis;
                        targetPoint = point;
                    }
                }

                points.emplace_back(targetPoint);
            }

        }


    }

    void Adaptive_sample_mid_in_vec_OPEN(int average_dis){
        //average_dis = 4;
        std::vector<std::pair<double,double>> distance_voxel_size = {
                {0.2,  0.2},
                {2.0,  0.8},
                {average_dis/2,   0.4},
                {1.5*average_dis,  1.6},
                {min(200,2*average_dis), -1.}
        };


        std::vector<tsl::robin_map<Voxel, std::vector<PointType>>> indices_map(distance_voxel_size.size());
        points.clear();

        for(auto &point : all_points){//do not use the & ,may bechange in point
//            double dis = std::sqrt(point.point[0] * point.point[0] + point.point[1] * point.point[1] +
//                    point.point[2] * point.point[2]);
            double dis = point.distance;
//            ROS_INFO("DIS: %f",dis);
            if(dis >= distance_voxel_size.front().first && dis < distance_voxel_size.back().first) {

                auto lw = std::lower_bound(distance_voxel_size.begin(), distance_voxel_size.end(), dis,
                                           [](const std::pair<double, double> &rhs, double lhs) {
                                               return rhs.first < lhs;
                                           });//first bigger

                auto index_ = std::distance(distance_voxel_size.begin(), lw) - 1;
                double voxel_size = distance_voxel_size[index_].second;

                Voxel voxel;
                voxel.x = static_cast<short>(point.point[0] / voxel_size);
                if(voxel.x<0) voxel.x--;
                voxel.y = static_cast<short>(point.point[1] / voxel_size);
                if(voxel.y<0) voxel.y--;
                voxel.z = static_cast<short>(point.point[2] / voxel_size);
                if(voxel.z<0) voxel.z--;

                indices_map[index_][voxel].emplace_back(point);
            }

        }
        for(const auto &grid_map : indices_map) {
            for(const auto & [_, points_vec] : grid_map){
                PointType midP;

                double min_dis = std::numeric_limits<double>::max();
                for(auto & point : points_vec){
                    midP.point += point.point;
                }
                midP.point /=(double) points_vec.size();

                PointType targetPoint;
                for(auto &point : points_vec){
                    double dis = (midP.point[0]-point.point[0])*(midP.point[0]-point.point[0])+
                                 (midP.point[1]-point.point[1])*(midP.point[1]-point.point[1])+
                                 (midP.point[2]-point.point[2])*(midP.point[2]-point.point[2]);
                    if(dis < min_dis){
                        min_dis = dis;
                        targetPoint = point;
                    }
                }

                points.emplace_back(targetPoint);
            }

        }


    }

    void updateWorldCloud(){
        for(auto & point_type : all_points){
            point_type.point_world = pose.linearInplote(point_type.alpha) * point_type.point;
        }
        return;
    }

    void updateFromDownSample(){

        for(auto & point_type : points){
            Eigen::Vector3d point = point_type.point;
            double alpha=(point_type.timestamp-timeStart)/(timeEnd-timeStart);
            SE3 temp_T_world=pose.linearInplote(alpha);
            point=temp_T_world * point;
            point_type.point_world = point;

        }
        return;
    }

    void deskew(){
        for(auto & point_type : all_points){
            Eigen::Vector3d point = point_type.point;
//            if(point.ring>20){
//                if(count % 5) continue;//20
//            }
//            else {
//                if(count % 50) continue;
//            }
            double alpha=(point_type.timestamp-timeStart)/(timeEnd-timeStart);
            SE3 temp_T_world=pose.linearInplote(alpha);
            point=pose.end_pose.inverse()*temp_T_world * point;
            point_type.point_deskew = point;

        }

    }

    void getWorldPoints(std::vector<PointType> & targetPoint){
        updateWorldCloud();
        targetPoint.insert(targetPoint.end(),all_points.begin(),all_points.end());
//        updateFromDownSample();
//        targetPoint.insert(targetPoint.end(),points.begin(),points.end());
    }

    void Reset(){

        headertime=timeStart=timeEnd=0;
        pose.initialMotion();
        points.clear();
        all_points.clear();
    }
    ~frame_info(){};

    void normalize(){
        pose.normalize();
    }
    
    V3D getEndTrans(){
        return pose.endTrans();
    }

    V3D getBeginTrans(){
        return pose.beginTrans();
    }

    Eigen::Quaterniond endQuat(){
        return pose.endQuat();
    }

    Eigen::Quaterniond beginQuat(){
        return pose.beginQuat();
    }

    M3D getEndRot(){
        return pose.endRot();
    }

    M3D getBeginRot(){
        return pose.endRot();
    }
};




