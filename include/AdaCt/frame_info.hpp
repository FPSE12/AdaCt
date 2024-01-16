#pragma once

#include "AdaCt/utility.h"
#include "AdaCt/optPose.hpp"
#include "voxelmap/voxelmap.hpp"

#define DOWNSAMPLE_VOXEL_SIZE 0.6
#define DOWNSAMPLE_EDGE_VOXEL_SIZE 0.2
#define DOWNSAMPLE_PLANE_VOXEL_SIZE 0.4

const bool pointcmp(PointXYZIRT &x, PointXYZIRT &y){return x.x * x.x + x.y*x.y+x.z*x.z < y.x*y.x+y.y*y.y+y.z*y.z;}

class frame_info
{
public:
    /* data */
    OptPose pose;
    double headertime, timeStart, timeEnd;
    int frame_id;
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

    frame_info(){
        downsampleLeafsize=0.2;
        voxelSize = DOWNSAMPLE_VOXEL_SIZE;
        blind=0.2;

        edgeDSsize=DOWNSAMPLE_EDGE_VOXEL_SIZE;
        planeDSsize=DOWNSAMPLE_PLANE_VOXEL_SIZE;

        pose.initialMotion();

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
    frame_info(double headertime_,double timeStart_, double timeEnd_, int frame_id_){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;

        pose.initialMotion();

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
    }

    void setFrameTime(double headertime_,double timeStart_, double timeEnd_, int frame_id_ ){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;
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


    void setMotion(const OptPose currpose){
        pose=currpose;
    }

    void setMotion(const Eigen::Quaterniond begin_quat, const Eigen::Quaterniond end_quat, 
                    const Eigen::Vector3d begin_trans, const Eigen::Vector3d end_trans ){
        pose.begin_pose=Sophus::SE3d(begin_quat, begin_trans);
        pose.end_pose=Sophus::SE3d(end_quat, end_trans);

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
//             int num = voxel_block.points.size();
//             if(num<5){
//                 continue;
//             }else if(num <10){
//
//             }

             PointXYZIRT midP = voxel_block.points[voxel_block.numPoints()/2];//use the const? cannot change the var in object(const auto used in above)
             //sort(voxel_block.points.begin(),voxel_block.points.end(),pointcmp);
             cloud_ori_downsample->points.push_back(midP);


         }

     }
     void grid_sample_feature(){
//        if(edge->points.size()<200){
//            pcl::copyPointCloud(*edge, *edgeDS);
//        }else{
//            downSampleEdge();
//        }
//        if(plane->points.size()<800){
//            pcl::copyPointCloud(*plane, *planeDS);
//        }else{
//            downSamplePlane();
//        }
        downSampleEdge();
        downSamplePlane();

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

            PointXYZIRT midP = voxel_block.points[0];//use the const? cannot change the var in object(const auto used in above)

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

    void grid_sample_mid(double downsample_size){
        tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;
        //grid.reserve(size_t(cloud_ori->size()));
        Voxel voxel;
        //int blind_voxel=ceil(blind/voxelSize);
        for(auto point : cloud_ori->points){
            //PointXYZIRT rawP = cloud_ori->points[i];
            //Eigen::Vector3d  raw_point(cloud_ori->points[i].x,cloud_ori->points[i].y,cloud_ori->points[i].z);
            //double timestamp = cloud_ori->points[i].timestamp;
            double dis = point.x * point.x + point.y * point.y + point.z * point.z;
            voxel.x = static_cast<short>(point.x / downsample_size);
            voxel.y = static_cast<short>(point.y / downsample_size);
            voxel.z = static_cast<short>(point.z / downsample_size);
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
        //cloud_ori_downsample->reserve(grid.size());
        for(const auto &[_,voxel_block] : grid) {
            //cloud_ori_downsample->points.push_back(point);

            PointXYZIRT midP = voxel_block.findCloseToMid();//use the const? cannot change the var in object(const auto used in above)

            cloud_ori_downsample->points.push_back(midP);
        }
        //ROS_INFO("DOWN:%d",cloud_ori_downsample->points.size());

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
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);
//
//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;
//
//            temp_P = pose.end_pose.inverse() * temp_P;

//            --------------------SE3----------
            SE3 temp_T_world=pose.linearInplote(alpha);
            SE3 temp_T_end = pose.end_pose.inverse() * temp_T_world;
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
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;


//            ---------------------SE3---------------------------
            SE3 temp_T_world=pose.linearInplote(alpha);
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
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;

//            ----------------SE3-----------------
            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;
//
            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points[i]=temp;
//
            temp_P = pose.end_pose.inverse() * temp_P;
//
            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            cloud_deskew->points[i]=temp;
        }
        return;
    }
    void updateWorldCloud(){
        cloud_world->clear();
        //cloud_world->resize(cloud_ori->size());
        int count=0;
        for(auto point : cloud_ori->points){
            count++;
            if(point.ring>20){
                if(count % 10) continue;//20
            }
            else {
                if(count % 50) continue;
            }
            PointXYZIRT temp=point;
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points.push_back(temp);


        }
        return;
    }
    void updateFromDownSample(){
        cloud_world->clear();
        cloud_world->resize(cloud_ori_downsample->size());
        cloud_deskew->clear();
        cloud_deskew->resize(cloud_ori_downsample->size());
        for(int i=0;i<cloud_ori_downsample->size();i++){
            PointXYZIRT temp=cloud_ori_downsample->points[i];
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

//            POSE inter_pose=pose.linearInplote(alpha);
//            V3D temp_P(temp.x,temp.y,temp.z);
//            temp_P = inter_pose.first * temp_P + inter_pose.second;
//
//
//            temp.x=temp_P[0];
//            temp.y=temp_P[1];
//            temp.z=temp_P[2];
//
//            //千万不能用push_back，会在size的基础上进行增加
//            cloud_world->points[i]=temp;
////
//            temp_P = pose.end_pose.inverse() * temp_P;
////
//            temp.x=temp_P[0];
//            temp.y=temp_P[1];
//            temp.z=temp_P[2];
//
//            cloud_deskew->points[i]=temp;

            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->points[i]=temp;

//            temp_P = pose.end_pose.inverse() * temp_P;
//
//            temp.x=temp_P[0];
//            temp.y=temp_P[1];
//            temp.z=temp_P[2];
//
//            cloud_deskew->points[i]=temp;
        }
        return;
    }

    void updateFeature(){
        updateEdgeFeature();
        updatePlaneFeature();
    }

    void updatePlaneFeature(){

        plane_world->clear();

        for(int i=0;i<plane->size();i++){
            PointXYZIRT temp=plane->points[i];
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            plane_world->push_back(temp);

//            temp_P = pose.end_pose.inverse() * temp_P;
//
//            temp.x=temp_P[0];
//            temp.y=temp_P[1];
//            temp.z=temp_P[2];
//
//            plane_deskew->points[i]=temp;
        }
        return;
    }

    void updateEdgeFeature(){

        edge_world->clear();
        for(int i=0;i<edge->size();i++){
            PointXYZIRT temp=edge->points[i];
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            //千万不能用push_back，会在size的基础上进行增加
            edge_world->push_back(temp);

//            temp_P = pose.end_pose.inverse() * temp_P;
//
//            temp.x=temp_P[0];
//            temp.y=temp_P[1];
//            temp.z=temp_P[2];
//
//            edge_deskew->points[i]=temp;
        }
        return;
    }

    void Reset(){

        headertime=timeStart=timeEnd=0;

        cloud_ori->clear();
        cloud_deskew->clear();
        cloud_world->clear();
        cloud_ori_downsample->clear();

        pose.initialMotion();

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
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

    Eigen::Quaterniond endQaut(){
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




