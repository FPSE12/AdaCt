#pragma once

#include "AdaCt/utility.h"
#include "AdaCt/optPose.hpp"
#include "voxelmap/voxelmap_OctoTree.hpp"
#include <opencv/cv.h>

#define DOWNSAMPLE_VOXEL_SIZE 0.4
#define DOWNSAMPLE_EDGE_VOXEL_SIZE 0.2
#define DOWNSAMPLE_PLANE_VOXEL_SIZE 0.4



const bool pointcmp(PointXYZIRT &x, PointXYZIRT &y){return x.x * x.x + x.y*x.y+x.z*x.z < y.x*y.x+y.y*y.y+y.z*y.z;}
const bool timelist(PointXYZIRT &x, PointXYZIRT &y){return x.timestamp < y.timestamp;}
class frame_info
{
public:
    /* data */
    OptPose pose;
    double headertime, timeStart, timeEnd;
    int frame_id;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_world;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori_downsample;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_deskew;

    pcl::PointCloud<PointXYZIRT>::Ptr ground_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr segment_cloud;

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

    std::vector<std::pair<double,double>> distance_voxel_size = {
            {0.2,  0.1},
            {2.0,  0.2},
            {4.,   0.4},
            {8.,   0.8},
            {16.,  1.6},
            {200., -1.}
    };

    tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;

public:

    frame_info(){

        voxelSize = DOWNSAMPLE_VOXEL_SIZE;
        blind=0.2;

        pose.initialMotion();

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());

        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
        ground_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
        segment_cloud.reset(new pcl::PointCloud<PointXYZIRT>());

        grid.clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        point_image.resize(N_SCAN,vector<PointXYZIRT *>(Horizon_SCAN, nullptr));

    };
    frame_info(double headertime_,double timeStart_, double timeEnd_, int frame_id_){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;

        pose.initialMotion();

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());
    }

    void setFrameTime(double headertime_,double timeStart_, double timeEnd_, int frame_id_ ){
        headertime=headertime_;
        timeStart=timeStart_;
        timeEnd=timeEnd_;
        frame_id=frame_id_;
    }

    void subSampleFrame(std::vector<PointType> & frame, double voxel_size){
        tsl::robin_map<Voxel, PointType> grid_map;

        for(auto point : frame){
            Voxel voxel;
            voxel.x = static_cast<short>(point.point[0] / voxel_size);
            if(voxel.x<0) voxel.x--;
            voxel.y = static_cast<short>(point.point[1] / voxel_size);
            if(voxel.y<0) voxel.y--;
            voxel.z = static_cast<short>(point.point[2] / voxel_size);
            if(voxel.z<0) voxel.z--;
            if(grid_map.find(voxel)!=grid_map.end()){
                grid_map[voxel] = point;
            }

        }
        frame.clear();
        for(const auto & [_, point] : grid_map){
            frame.push_back(point);
        }

    }

    void setOricloud(pcl::PointCloud<PointXYZIRT>::ConstPtr ori_cloud){
        pcl::copyPointCloud(*ori_cloud, *cloud_ori);
        //ROS_INFO("%f", cloud_ori->points[1].timestamp);
        cloud_world->clear();
        cloud_ori_downsample->clear();
        points.clear();
        all_points.clear();
        timeVec.resize(cloud_ori->size());
        //add to all points
        for(auto point : cloud_ori->points){
            if(!std::isfinite(point.x)|| !std::isfinite(point.y) || !std::isfinite(point.z)){
                continue;
            }
            double dis = std::sqrt( point.x * point.x + point.y * point.y + point.z * point.z);
            if(dis<blind){
                continue;
            }

            PointType p;
            p.point<<point.x, point.y, point.z;
            p.timestamp = point.timestamp;
            p.alpha = (p.timestamp - timeStart)/(timeEnd - timeStart);
            p.intensity = point.intensity;
            all_points.push_back(p);
        }

        //subsample for add to map
        std::mt19937_64 g;
        std::shuffle(all_points.begin(),all_points.end(),g);
        //std::shuffle(cloud_ori->points.begin(),cloud_ori->points.end(),g);
        subSampleFrame(all_points, 0.05);



    }


    void setMotion(const OptPose currpose){
        pose=currpose;
    }

    void setMotion(const Eigen::Quaterniond begin_quat, const Eigen::Quaterniond end_quat, 
                    const Eigen::Vector3d begin_trans, const Eigen::Vector3d end_trans ){
        pose.begin_pose=Sophus::SE3d(begin_quat, begin_trans);
        pose.end_pose=Sophus::SE3d(end_quat, end_trans);

    }


    void getTimeStamp(double & header,double & time_begin, double & time_end){
        sort(cloud_ori->points.begin(),cloud_ori->points.end(), timelist);
        headertime = header;

        timeStart = cloud_ori->points[0].timestamp;
        timeEnd = cloud_ori->points.back().timestamp;

        //header = headertime;
        time_begin = timeStart;
        time_end = timeEnd;
    }

    void findRingStartEnd(){
        //always 57600
        //ROS_INFO("ori_point: %d", cloud_ori->points.size());

        first_line.clear();
        last_line.clear();
        timestamp_line.clear();
        segment_cloud->clear();
        int point_num = cloud_ori->points.size();
        //i:0-32, high to low, ring:31-0
        for(int i=0;i< N_SCAN;i++){
            int j = point_num - N_SCAN + i;

            //scan way is  a problem....

            Eigen::Vector3d first = Eigen::Vector3d (cloud_ori->points[i].x,cloud_ori->points[i].y,cloud_ori->points[i].z);
            Eigen::Vector3d last =  Eigen::Vector3d (cloud_ori->points[j].x,cloud_ori->points[j].y,cloud_ori->points[j].z);
            if(!std::isfinite(first[0]) || !std::isfinite(first[1]) || !std::isfinite(first[2])
               || !std::isfinite(last[0]) || !std::isfinite(last[1]) || !std::isfinite(last[2])||
               cloud_ori->points[i].ring!= cloud_ori->points[j].ring){
                continue;
            }

            segment_cloud->push_back(cloud_ori->points[i]);
//            segment_cloud->push_back(cloud_ori->points[j]);
//            ROS_INFO("TIMESTAMP: %f, %f; ring : %d, %d", cloud_ori->points[i].timestamp, cloud_ori->points[j].timestamp,
//                     cloud_ori->points[i].ring, cloud_ori->points[j].ring);

            timestamp_line.push_back(pair<double,double>(cloud_ori->points[i].timestamp,cloud_ori->points[j].timestamp));
            first_line.push_back(first);
            last_line.push_back(last);

        }
//        ROS_INFO("CORRESPONSE: %d",first_line.size());
    }



    void Project2Img(){
        double horizonAngle ,range;
        size_t rowIdn, columnIdn, index;
        for(auto & point: cloud_ori->points){
            if(!std::isfinite(point.x)|| !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            rowIdn = point.ring;

            if(rowIdn<0 || rowIdn>N_SCAN) continue;

            // 0-360 -> 0->horizon, 0 angle not corresponse begin
            //atan2(y,x)
            horizonAngle = atan2(point.y, point.x) * 180 / M_PI;

            if(horizonAngle<0) horizonAngle += 360;

            columnIdn = round(horizonAngle/ang_res_x);

//            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
//            if (columnIdn >= Horizon_SCAN)
//                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(point.x*point.x + point.y* point.y + point.z* point.z );

            if(range<blind) continue;

            rangeMat.at<float>(rowIdn,columnIdn)=range;

            point_image[rowIdn][columnIdn] = &point;
            index = columnIdn + rowIdn * Horizon_SCAN;

        }
        //see 0 and 360 point
//        for(int i=0;i<N_SCAN;i++){
//            ROS_INFO("RING: %d--------------------", i);
//            if(point_image[i][0]!= nullptr){
//                ROS_INFO("start:  %f, %f", point_image[i][0]->timestamp, rangeMat.at<float>(i,0));
//                //segment_cloud->push_back(*point_image[i][0]);
//            }
//            if(point_image[i][Horizon_SCAN-1]!= nullptr){
//                ROS_INFO("END:  %f,%f", point_image[i][Horizon_SCAN-1]->timestamp, rangeMat.at<float>(i,Horizon_SCAN-1));
//                //segment_cloud->push_back(*point_image[i][Horizon_SCAN-1]);
//            }
//
//
//        }
    }

    void Segment(){
        auto begin=std::chrono::steady_clock::now();
        point_image.clear();
        point_image.resize(N_SCAN,vector<PointXYZIRT *>(Horizon_SCAN, nullptr));
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        ground_cloud->clear();
        segment_cloud->clear();
        //project to image
        Project2Img();

        //ground_remove

        auto end = std::chrono::steady_clock::now();
        ROS_INFO("SEG COST: %f ms",
                 std::chrono::duration<double, std::milli>(end - begin).count()
                 );
    }

    void gird_sample_mid_in_vector(double downsample_size){
        points.clear();
        std::vector<PointType> frame_sub;
//        frame_sub.resize(frame.size());
        for(auto point : all_points){
            frame_sub.push_back(point);
        }
        subSampleFrame(frame_sub, downsample_size);
        for(auto point : frame_sub){
            points.push_back(point);
        }
    }

    void  grid_sample_mid_in_pcl(double downsample_size){
        cloud_ori_downsample->clear();
        points.clear();
        all_points.clear();

        grid.clear();

        //tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>> grid;

        //grid.reserve(size_t(cloud_ori->size()));
        Voxel voxel;
        //int blind_voxel=ceil(blind/voxelSize);
        for(auto  point : cloud_ori->points){
            //PointXYZIRT rawP = cloud_ori->points[i];
            //Eigen::Vector3d  raw_point(cloud_ori->points[i].x,cloud_ori->points[i].y,cloud_ori->points[i].z);
            //double timestamp = cloud_ori->points[i].timestamp;
            if(!std::isfinite(point.x)|| !std::isfinite(point.y) || !std::isfinite(point.z)){
                continue;
            }
            double dis = std::sqrt( point.x * point.x + point.y * point.y + point.z * point.z);
            voxel.x = static_cast<short>(point.x / downsample_size);
            if(voxel.x<0) voxel.x--;
            voxel.y = static_cast<short>(point.y / downsample_size);
            if(voxel.y<0) voxel.y--;
            voxel.z = static_cast<short>(point.z / downsample_size);
            if(voxel.z<0) voxel.z--;
            if(dis< blind){
                continue;
            }

            grid[voxel].addPoint(point);
        }


        for(const auto &[_,voxel_block] : grid) {


            PointXYZIRT midP = voxel_block.findCloseToMid();//use the const? cannot change the var in object(const auto used in above)

            cloud_ori_downsample->points.push_back(midP);

            PointType p;
            p.point<<midP.x, midP.y, midP.z;
            p.timestamp = midP.timestamp;
            p.alpha = (p.timestamp - timeStart)/(timeEnd - timeStart);
            p.intensity = midP.intensity;
            points.push_back(p);
        }
        //ROS_INFO("DOWN:%d",cloud_ori_downsample->points.size());

    }


    void Adaptive_sample_mid_in_pcl(){
        std::vector<tsl::robin_map<Voxel, VoxelBlock<PointXYZIRT>>> indices_map(distance_voxel_size.size());
        points.clear();
        cloud_ori_downsample->clear();
        for(auto point : cloud_ori->points){//do not use the & ,may bechange in point
            if(!std::isfinite(point.x)|| !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(point.timestamp)){
                continue;
            }
            double dis = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
//            ROS_INFO("DIS: %f",dis);
            if(dis >= distance_voxel_size.front().first && dis < distance_voxel_size.back().first) {

                auto lw = std::lower_bound(distance_voxel_size.begin(), distance_voxel_size.end(), dis,
                                           [](const std::pair<double, double> &rhs, double lhs) {
                                               return rhs.first < lhs;
                                           });

                auto index_ = std::distance(distance_voxel_size.begin(), lw) - 1;
                double voxel_size = distance_voxel_size[index_].second;

                Voxel voxel;
                voxel.x = static_cast<short>(point.x / voxel_size);
                voxel.y = static_cast<short>(point.y / voxel_size);
                voxel.z = static_cast<short>(point.z / voxel_size);

                indices_map[index_][voxel].addPoint(point);
            }

        }
        for(const auto &grid_map : indices_map) {
            for(const auto & [_, voxelblock] : grid_map){
                PointXYZIRT midP = voxelblock.findCloseToMid();
                cloud_ori_downsample->points.push_back(midP);

                PointType p;
                p.point<<midP.x, midP.y, midP.z;
                p.timestamp = midP.timestamp;
                p.alpha = (p.timestamp - timeStart)/(timeEnd - timeStart);
                if(!isfinite(p.alpha)){
                    ROS_ERROR("alpha wrong! %f",p.alpha);//-----------error-------
                    ROS_ERROR("time_start: %f, timeEnd: %f, timestamp:%f", timeStart, timeEnd, p.timestamp);
                }
                p.intensity = midP.intensity;
                points.push_back(p);
            }

        }


    }

    void updateWorldCloud(){
        cloud_world->clear();
        all_points.clear();
        //cloud_world->resize(cloud_ori->size());
        int count=0;
        for(auto & point : cloud_ori->points){
            count++;
            if(count%5|| !isfinite(point.x) || !isfinite(point.y) || !isfinite(point.z)){
                continue;
            }
//            if(point.ring>20){
//                if(count % 5) continue;//20
//            }
//            else {
//                if(count % 50) continue;
//            }
            PointXYZIRT temp=point;
            double alpha=(temp.timestamp-timeStart)/(timeEnd-timeStart);

            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(temp.x,temp.y,temp.z);
            temp_P=temp_T_world * temp_P;

            temp.x=temp_P[0];
            temp.y=temp_P[1];
            temp.z=temp_P[2];

            cloud_world->points.push_back(temp);

            PointType p;
            p.point<<point.x,point.y,point.z;
            p.point_world<<temp_P[0], temp_P[1], temp_P[2];
            p.intensity = temp.intensity;
            all_points.push_back(p);

        }
        return;
    }

    void updateFromDownSample(){
        cloud_world->clear();
//        cloud_world->resize(cloud_ori_downsample->size());
//        cloud_deskew->clear();
//        cloud_deskew->resize(cloud_ori_downsample->size());
        for(int i=0;i<cloud_ori_downsample->size();i++){
            PointXYZIRT point = cloud_ori_downsample->points[i];
            double alpha=(point.timestamp-timeStart)/(timeEnd-timeStart);

            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(point.x,point.y,point.z);
            temp_P=temp_T_world * temp_P;

            point.x=temp_P[0];
            point.y=temp_P[1];
            point.z=temp_P[2];

            //update points for update map
            points[i].point_world<<temp_P[0],temp_P[1],temp_P[2];
            //千万不能用push_back，会在size的基础上进行增加
            cloud_world->push_back(point);


        }
        return;
    }

    void deskew(){
        cloud_deskew->clear();

        for(auto point : cloud_ori_downsample->points){
            double alpha=(point.timestamp-timeStart)/(timeEnd-timeStart);
            SE3 temp_T_world=pose.linearInplote(alpha);
            V3D temp_P(point.x,point.y,point.z);
            temp_P=pose.begin_pose.inverse() *temp_T_world * temp_P;
            point.x=temp_P[0];
            point.y=temp_P[1];
            point.z=temp_P[2];
            cloud_deskew->push_back(point);
        }
    }


    void Reset(){

        headertime=timeStart=timeEnd=0;

        cloud_ori->clear();
        cloud_world->clear();
        cloud_ori_downsample->clear();
        pose.initialMotion();
        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori_downsample.reset(new pcl::PointCloud<PointXYZIRT>());

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




