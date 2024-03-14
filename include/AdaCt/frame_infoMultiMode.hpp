#pragma once

#include "AdaCt/utility.h"
#include "AdaCt/optPoseMultiMode.hpp"
#include "voxelmap/voxelmap.hpp"





class frame_info
{
public:
    /* data */
    //OptPose pose;

    //SE3 begin_pose, middle_pose, end_pose;
    OptPoseMultiMode poses;
    double headertime;
    std::vector<double> poses_time;

    double timeStart,timeEnd;
    int frame_id;
    int node_num;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_deskew;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_world;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori_downsample;

    pcl::PointCloud<PointXYZIRT>::Ptr edge;
    pcl::PointCloud<PointXYZIRT>::Ptr plane;
    pcl::PointCloud<PointXYZIRT>::Ptr edge_world;
    pcl::PointCloud<PointXYZIRT>::Ptr plane_world;
    pcl::PointCloud<PointXYZIRT>::Ptr edge_deskew;
    pcl::PointCloud<PointXYZIRT>::Ptr plane_deskew;
    pcl::PointCloud<PointXYZIRT>::Ptr edgeDS;
    pcl::PointCloud<PointXYZIRT>::Ptr planeDS;

    pcl::VoxelGrid<PointXYZIRT> downSamplefliter;

    std::vector<double> timeVec;
    // pcl::VoxelGrid<PointType> downSamplefliter;
    double  downsampleLeafsize;
    double voxelSize;
    double edgeDSsize;
    double planeDSsize;
    double blind;

public:

    frame_info(int node_num_=2):node_num(node_num_){
        downsampleLeafsize=0.2;
        voxelSize = 0.5;
        blind=0.2;

        edgeDSsize=0.2;
        planeDSsize=0.4;

        poses.init(node_num);

        poses_time.resize(node_num_);

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
        edge.reset(new pcl::PointCloud<PointXYZIRT>());
        plane.reset(new pcl::PointCloud<PointXYZIRT>());

        edge_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        edge_world.reset(new pcl::PointCloud<PointXYZIRT>());
        plane_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        plane_world.reset(new pcl::PointCloud<PointXYZIRT>());
        edgeDS.reset(new pcl::PointCloud<PointXYZIRT>());
        planeDS.reset(new pcl::PointCloud<PointXYZIRT>());
    };

    void setNode(int num){
        node_num=num;
        poses.init(node_num);

        poses_time.resize(node_num,0);
    }

    void setFrameTime(double headertime_,vector<double> time, int frame_id_ ){
        headertime=headertime_;
        frame_id=frame_id_;

        poses_time=time;
        timeStart=poses_time.front();
        timeEnd=poses_time.back();

    }

    void setOricloud(pcl::PointCloud<PointXYZIRT>::ConstPtr ori_cloud){
        pcl::copyPointCloud(*ori_cloud, *cloud_ori);
        //ROS_INFO("%f", cloud_ori->points[1].timestamp);
        cloud_deskew->clear();
        cloud_world->clear();
        cloud_ori_downsample->clear();
        timeVec.resize(cloud_ori->size());
//        cloud_deskew->resize(cloud_ori->size());
//        cloud_world->resize(cloud_ori->size());
//        cloud_ori_downsample->resize(cloud_ori->size());
    }

    void setFeaturecloud(pcl::PointCloud<PointXYZIRT>::ConstPtr edge_, pcl::PointCloud<PointXYZIRT>::Ptr plane_){
        pcl::copyPointCloud(*edge_, *edge);
        pcl::copyPointCloud(*plane_,*plane);
    }


//    void setMotion(const OptPose currpose){
//        pose=currpose;
//    }

    void setMotion(const std::vector<Eigen::Quaterniond> quats,
                   const std::vector<Eigen::Vector3d> trans
                    ){
        if(quats.size() != trans.size() || quats.size()!= node_num){
            ROS_ERROR("NUMS DO NOT MATCH!");

        }
        for(int i=0;i<node_num;i++){
            //note
            poses[i]=Sophus::SE3(quats[i],trans[i]);
        }

    }


//    void downSampleOriCloud(){
//        //ROS_INFO("%s",downSamplefliter.getFilterFieldName().c_str());
//        cloud_ori_downsample->clear();
//
//        downSamplefliter.setLeafSize(downsampleLeafsize,downsampleLeafsize,downsampleLeafsize);
//        downSamplefliter.setInputCloud(cloud_ori);
//
//        downSamplefliter.filter(*cloud_ori_downsample);//downsample wouldn't process timestamp（voxel middle ）
////        ROS_INFO("%f",cloud_ori->points[1].x);
//        ROS_INFO("%f",cloud_ori_downsample->points[1].ring);
//        ROS_INFO("%lu, %lu",cloud_ori->size(),cloud_ori_downsample->size());
//
//    }


     //new version，，cannot_used
     void grid_sample_timeMid(){
         tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;
         grid.reserve(size_t(cloud_ori->size()));
         Voxel voxel;
         //int blind_voxel=ceil(blind/voxelSize);
         for(int i=0;i<cloud_ori->size();i++){
             PointXYZIRT rawP = cloud_ori->points[i];
             Eigen::Vector3d  raw_point(cloud_ori->points[i].x,cloud_ori->points[i].y,cloud_ori->points[i].z);
             double timestamp = cloud_ori->points[i].timestamp;
             double dis = raw_point.norm();
             voxel.x = static_cast<short>(cloud_ori->points[i].x / voxelSize);
             voxel.y = static_cast<short>(cloud_ori->points[i].y / voxelSize);
             voxel.z = static_cast<short>(cloud_ori->points[i].z / voxelSize);
             if(dis< blind || !std::isfinite(raw_point(0))|| !std::isfinite(raw_point(1)) || !std::isfinite(raw_point(2))){
                 continue;
             }
             if(grid.find(voxel)==grid.end()){
                 grid[voxel].addPoint(rawP);
             }else{
                 grid[voxel].addPoint(rawP);
             }
         }
         cloud_ori_downsample->clear();
         cloud_ori_downsample->reserve(grid.size());
         for(const auto &[_,voxel_block] : grid){
             //cloud_ori_downsample->points.push_back(point);

             PointXYZIRT midP = voxel_block.points[voxel_block.numPoints()/2];//use the const? cannot change the var in object(const auto used in above)

             cloud_ori_downsample->points.push_back(midP);
         }

     }
     void grid_sample_feature(){
        if(edge->points.size()<200){
            pcl::copyPointCloud(*edge, *edgeDS);
        }else{
            downSampleEdge();
        }
        if(plane->points.size()<800){
            pcl::copyPointCloud(*plane, *planeDS);
        }else{
            downSamplePlane();
        }

    }

    void downSampleEdge(){
        tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;
        grid.reserve(size_t(edge->size()));
        Voxel voxel;
        //int blind_voxel=ceil(blind/voxelSize);
        for(int i=0;i<edge->size();i++){
            PointXYZIRT rawP = edge->points[i];
            Eigen::Vector3d  raw_point(edge->points[i].x,edge->points[i].y,edge->points[i].z);
            double timestamp = edge->points[i].timestamp;
            double dis = raw_point.norm();
            voxel.x = static_cast<short>(edge->points[i].x / edgeDSsize);
            voxel.y = static_cast<short>(edge->points[i].y / edgeDSsize);
            voxel.z = static_cast<short>(edge->points[i].z / edgeDSsize);
            if(dis< blind || !std::isfinite(raw_point(0))|| !std::isfinite(raw_point(1)) || !std::isfinite(raw_point(2))){
                continue;
            }
            if(grid.find(voxel)==grid.end()){
                grid[voxel].addPoint(rawP);
            }else{
                grid[voxel].addPoint(rawP);
            }
        }
        edgeDS->clear();
        edgeDS->reserve(grid.size());
        for(const auto &[_,voxel_block] : grid){
            //cloud_ori_downsample->points.push_back(point);

            PointXYZIRT midP = voxel_block.findCloseToMid();//use the const? cannot change the var in object(const auto used in above)

            edgeDS->points.push_back(midP);
        }

    }

    void downSamplePlane(){
        tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;
        grid.reserve(size_t(plane->size()));
        Voxel voxel;
        //int blind_voxel=ceil(blind/voxelSize);
        for(int i=0;i<plane->size();i++){
            PointXYZIRT rawP = plane->points[i];
            Eigen::Vector3d  raw_point(plane->points[i].x,plane->points[i].y,plane->points[i].z);
            double timestamp = plane->points[i].timestamp;
            double dis = raw_point.norm();
            voxel.x = static_cast<short>(plane->points[i].x / planeDSsize);
            voxel.y = static_cast<short>(plane->points[i].y / planeDSsize);
            voxel.z = static_cast<short>(plane->points[i].z / planeDSsize);
            if(dis< blind || !std::isfinite(raw_point(0))|| !std::isfinite(raw_point(1)) || !std::isfinite(raw_point(2))){
                continue;
            }
            if(grid.find(voxel)==grid.end()){
                grid[voxel].addPoint(rawP);
            }else{
                grid[voxel].addPoint(rawP);
            }
        }
        planeDS->clear();
        planeDS->reserve(grid.size());
        for(const auto &[_,voxel_block] : grid){
            //cloud_ori_downsample->points.push_back(point);

            PointXYZIRT midP = voxel_block.findCloseToMid();//use the const? cannot change the var in object(const auto used in above)

            planeDS->points.push_back(midP);
        }

    }

    void grid_sample_mid(double DownSampleSize){
        tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;
        //grid.reserve(size_t(cloud_ori->size()));
        Voxel voxel;
        //int blind_voxel=ceil(blind/voxelSize);
        for(auto point : cloud_ori->points){
//            Eigen::Vector3d  raw_point(point.x,point.y,point.z);
//            double timestamp = cloud_ori->points[i].timestamp;
            double dis = point.x*point.x + point.y*point.y + point.z * point.z;
            voxel.x = static_cast<short>(point.x / DownSampleSize);
            voxel.y = static_cast<short>(point.y / DownSampleSize);
            voxel.z = static_cast<short>(point.z / DownSampleSize);
            if(dis< blind || !std::isfinite(point.x)|| !std::isfinite(point.y) || !std::isfinite(point.z)){
                continue;
            }
//            if(grid.find(voxel)==grid.end()){
//                grid[voxel].addPoint(rawP);
//            }else{
//               grid[voxel].addPoint(rawP);
//            }
            grid[voxel].addPoint(point);
        }
        cloud_ori_downsample->clear();
        for(const auto &[_,voxel_block] : grid){
            //cloud_ori_downsample->points.push_back(point);

            PointXYZIRT midP = voxel_block.findCloseToMid();//use the const? cannot change the var in object(const auto used in above)

            cloud_ori_downsample->points.push_back(midP);
        }

    }

    void grid_sample(){
        tsl::robin_map<Voxel, PointXYZIRT> grid;
        grid.reserve(size_t(cloud_ori->size()));
        Voxel voxel;
        int blind_voxel=ceil(blind/voxelSize);
        for(int i=0;i<cloud_ori->size();i++){
            voxel.x = static_cast<short>(cloud_ori->points[i].x / voxelSize);
            voxel.y = static_cast<short>(cloud_ori->points[i].y / voxelSize);
            voxel.z = static_cast<short>(cloud_ori->points[i].z / voxelSize);
            if(voxel.x<blind_voxel && voxel.y<blind_voxel && voxel.z<blind_voxel){
                continue;
            }
            if(grid.find(voxel)==grid.end()){
                grid[voxel]=cloud_ori->points[i];
            }
        }
        cloud_ori_downsample->clear();
        cloud_ori_downsample->reserve(grid.size());
        for(const auto &[_,point] : grid){
            cloud_ori_downsample->points.push_back(point);
        }

    }
   
    //use opt_pose to change cloud_ori to cloud_deskew



    void Distortion(){
        cloud_deskew->clear();
        cloud_deskew->resize(cloud_ori->size());
        for(int i=0;i<cloud_ori->size();i++){
            PointXYZIRT temp=cloud_ori->points[i];
            //ROS_INFO("%f, %f,%f", cloud_ori->points[i].timestamp, timeStart, timeEnd);
            double alpha=(temp.timestamp-poses_time.back())/(poses_time.back()-poses_time.front());
//
//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;
//
//            temp_P = pose.end_pose.inverse() * temp_P;

//            --------------------SE3----------
            SE3 temp_T_world=poses.linearInplote(alpha);
            SE3 temp_T_end =poses.back().inverse() * temp_T_world;
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_end * temp_P;//to the end
//
            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            cloud_deskew->points[i]=temp;
        }
        return;
    }

    void TranstoWorld(){
        cloud_world->clear();
        cloud_world->resize(cloud_ori->size());
        for(int i=0;i<cloud_ori->size();i++){
            PointXYZIRT temp=cloud_ori->points[i];
            double alpha=(temp.timestamp-poses_time.back())/(poses_time.back()-poses_time.front());

//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;


//            ---------------------SE3---------------------------
            SE3 temp_T_world=poses.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;
//
            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points[i]=temp;
        }
        return;
    }

    void update(){
        cloud_world->clear();
        cloud_world->resize(cloud_ori->size());
        cloud_deskew->clear();
        cloud_deskew->resize(cloud_ori->size());
        for(int i=0;i<cloud_ori->size();i++){
            PointXYZIRT temp=cloud_ori->points[i];
            double alpha=(temp.timestamp-poses_time.back())/(poses_time.back()-poses_time.front());

//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;

//            ----------------SE3-----------------
            SE3 temp_T_world=poses.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;
//
            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points[i]=temp;
//
            temp_P = poses.back().inverse() * temp_P;
//
            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            cloud_deskew->points[i]=temp;
        }
        return;
    }

    void updateFromDownSample(){
        cloud_world->clear();
        for(auto point : cloud_ori_downsample->points){

            double alpha=(point.timestamp-poses_time.front())/(poses_time.back()-poses_time.front());


            SE3 temp_T_world=poses.linearInplote(alpha);
            V3D temp_P(point.x,point.y,point.z);
            temp_P=temp_T_world * temp_P;

            point.x=temp_P[0];
            point.y=temp_P[1];
            point.z=temp_P[2];

            cloud_world->points.push_back(point);

        }
        return;
    }

    void updateFeature(){
        updateEdgeFeature();
        updatePlaneFeature();
    }

    void updatePlaneFeature(){
        plane_deskew->clear();
        plane_deskew->resize(planeDS->size());
        plane_world->clear();
        plane_world->resize(planeDS->size());
        for(int i=0;i<planeDS->size();i++){
            PointXYZIRT temp=planeDS->points[i];
            double alpha=(temp.timestamp-poses_time.back())/(poses_time.back()-poses_time.front());

            SE3 temp_T_world=poses.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            plane_world->points[i]=temp;

            temp_P = poses.back().inverse() * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            plane_deskew->points[i]=temp;
        }
        return;
    }

    void updateEdgeFeature(){
        edge_deskew->clear();
        edge_deskew->resize(edgeDS->size());
        edge_world->clear();
        edge_world->resize(edgeDS->size());
        for(int i=0;i<edgeDS->size();i++){
            PointXYZIRT temp=edgeDS->points[i];
            double alpha=(temp.timestamp-poses_time.back())/(poses_time.back()-poses_time.front());

            SE3 temp_T_world=poses.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            edge_world->points[i]=temp;

            temp_P = poses.back().inverse() * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            edge_deskew->points[i]=temp;
        }
        return;
    }

    void Reset(){

        headertime=0;

        poses_time.clear();
        poses.clear();

        cloud_ori->clear();
        cloud_deskew->clear();
        cloud_world->clear();
        cloud_ori_downsample->clear();



        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
    }
    ~frame_info(){};

    void normalize(){
        poses.normalize();
    }
    
    V3D getEndTrans(){
        return poses.back().translation();
    }

    V3D getBeginTrans(){
        return poses.front().translation();
    }

    Eigen::Quaterniond endQaut(){
        return poses.back().unit_quaternion();
    }

    Eigen::Quaterniond beginQuat(){
        return poses.front().unit_quaternion();
    }

    void Propagate(OptPoseMultiMode last){
        if(last.size()!=poses.size()){
            ROS_WARN("last key_poses cannot match curr");
            return;
        }
        poses[0]=last.back();
        for(int i=1;i<poses.size();i++){
            poses[i]=poses[i-1]*(last[i-1].inverse()*last[i]);
        }
    }
};




