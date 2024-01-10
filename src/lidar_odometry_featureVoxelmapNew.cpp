//
// Created by wjj on 2023/11/17.
//
#include "AdaCt/utility.h"
#include "AdaCt/frame_info.hpp"

//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap_NoProperty.hpp"
//#include "voxelmap/voxelmap.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;


const bool timelist(PointXYZIRT &x, PointXYZIRT &y){return x.timestamp < y.timestamp;}

//



class Lidar_odo : public configParam
{
public:
    ros::Subscriber full_sub;

    ros::Subscriber edge_sub;
    ros::Subscriber plane_sub;
    ros::Subscriber odo_Init;

    ros::Publisher map_pub;
    ros::Publisher odo_pub;
    ros::Publisher traj_pub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_ori_pub;
    ros::Publisher cloud_valid_pub;
    ros::Publisher cloud_world_pub;


    // cloud
    std::deque<sensor_msgs::PointCloud2> fullCloudQue;
    std::deque<sensor_msgs::PointCloud2> edgeCloudQue;
    std::deque<sensor_msgs::PointCloud2> planesCloudQue;
    std::deque<nav_msgs::Odometry> odoInitQue;

    sensor_msgs::PointCloud2 cloud_full;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;

    //features
    sensor_msgs::PointCloud2 cloud_edge;
    sensor_msgs::PointCloud2 cloud_plane;
    pcl::PointCloud<PointXYZIRT>::Ptr edge;
    pcl::PointCloud<PointXYZIRT>::Ptr lastEdge;
    pcl::PointCloud<PointXYZIRT>::Ptr plane;
    pcl::PointCloud<PointXYZIRT>::Ptr lastPlane;
    pcl::PointCloud<PointXYZIRT>::Ptr validEdge;
    pcl::PointCloud<PointXYZIRT>::Ptr validPlane;

    pcl::PointCloud<PointXYZIRT>::Ptr features;
    pcl::PointCloud<PointXYZIRT>::Ptr last_features;
    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;
    pcl::PointCloud<PointXYZIRT>::Ptr edgeGlobalMap;
    pcl::PointCloud<PointXYZIRT>::Ptr planeGlobalMap;

    //global traj
    nav_msgs::Path globalPath;

    double timeStart, timeEnd, headertime;
    frame_info curr_frame;

    std::mutex mtx;

    bool first_flag;
    int frame_count;
    int opt_edge_num;
    int opt_plane_num;

    // debug info
    int beforeRemoveNan, afterRomoveNan;

    // map
    //Voxelmap<PointXYZIRT> local_map;
    VoxelmapNoProperty<PointXYZIRT> edge_map;
    VoxelmapNoProperty<PointXYZIRT> plane_map;
//    Voxelmap<PointXYZIRT> feature_map;
    // opt pose
//    OptPose pre_pose, curr_pose;

    std::vector<OptPose> poses;
    SE3 last2curr;
    int iter_nums; // 5

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    Lidar_odo()
    {
        //ROS_INFO("1");
        full_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_odo::lidarCallback, this);
        edge_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 5, &Lidar_odo::edgeCallback, this);
        plane_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 5, &Lidar_odo::planeCallback, this);
        odo_Init = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &Lidar_odo::OdoCallBack, this);

        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/deskew_pointcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);
        cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/world_points",1);

        edge.reset(new pcl::PointCloud<PointXYZIRT>());
        plane.reset(new pcl::PointCloud<PointXYZIRT>());
        lastEdge.reset(new pcl::PointCloud<PointXYZIRT>());
        lastPlane.reset(new pcl::PointCloud<PointXYZIRT>());
        validEdge.reset(new pcl::PointCloud<PointXYZIRT>());
        validPlane.reset(new pcl::PointCloud<PointXYZIRT>());

        features.reset(new pcl::PointCloud<PointXYZIRT>());
        last_features.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());

        first_flag = true;
        frame_count =-1;

        iter_nums =5;

        fullCloudQue.clear();
        edgeCloudQue.clear();
        planesCloudQue.clear();

//        pre_pose.initialMotion();
//        curr_pose.initialMotion();

        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());
        planeGlobalMap.reset(new pcl::PointCloud<PointXYZIRT>());
        edgeGlobalMap.reset(new pcl::PointCloud<PointXYZIRT>());

        laserOdometryTrans.frame_id_ = "map";
        laserOdometryTrans.child_frame_id_ = "odom";
    }

    void OdoCallBack(const nav_msgs::Odometry::ConstPtr &odomsg){
        std::lock_guard<std::mutex> lock(mutex);
        odoInitQue.push_back(*odomsg);


    }

    void edgeCallback(const sensor_msgs::PointCloud2ConstPtr & edgeMsg){
        std::lock_guard<std::mutex> lock(mtx);
        edgeCloudQue.push_back(*edgeMsg);
    }

    void planeCallback(const sensor_msgs::PointCloud2ConstPtr & planeMsg){
        std::lock_guard<std::mutex> lock(mtx);
        planesCloudQue.push_back(*planeMsg);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        fullCloudQue.push_back(*laserMsg);
    }


    void updateInitMotion(){

    }

    void Initframe()
    {

        curr_frame.setFrameTime(headertime, timeStart, timeEnd, frame_count);
        curr_frame.setOricloud(cloud_ori);
        if(frame_count<=1){
            curr_frame.pose.initialMotion();
            //poses.push_back(curr_frame.pose);
        }else{
            curr_frame.pose.begin_pose = poses.back().end_pose;
//            //curr_frame.pose.end_pose = curr_frame.pose.begin_pose *(poses.back().begin_pose.inverse()*poses.back().end_pose);
//            curr_frame.pose.end_pose =curr_frame.pose.begin_pose *(poses.back().begin_pose.inverse()*poses.back().end_pose);
            curr_frame.pose.end_pose =curr_frame.pose.begin_pose *last2curr;
        }//need init with the curr cloud with last cloud

        //curr_frame.downSampleOriCloud();
        //下采样会导致问题
        curr_frame.setFeaturecloud(edge,plane);
        curr_frame.grid_sample_feature();

        //curr_frame.updateFeature();
    }

    void AddEdgeProblem(ceres::Problem &problem, ceres::LossFunction *loss_function,Eigen::Quaterniond &begin_quat, Eigen::Quaterniond &end_quat, Eigen::Vector3d & begin_trans, Eigen::Vector3d & end_trans){
        validEdge->clear();

        for(int i=0;i<curr_frame.edgeDS->size();i++){
            PointXYZIRT raw_point=curr_frame.edgeDS->points[i];
            double alpha = (raw_point.timestamp -timeStart)/(timeEnd -timeStart);
            // Eigen::Vector3d  sensor_location = (1- alpha) * begin_trans + alpha * end_trans;
            VoxelBlock<PointXYZIRT> neighbor;
            Eigen::Vector4d pabcd;
            Eigen::Vector3d curr_point(raw_point.x,raw_point.y,raw_point.z);
            if(edge_map.NeighborSearchEstiPlane(curr_frame.edge_world->points[i],0.8,pabcd)) {

                opt_edge_num++;
                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor, 1, 4, 4, 3, 3>(
                                new CTFunctor(alpha, curr_point, pabcd, 1.0));
                problem.AddResidualBlock(cost_function,
                                         loss_function,
                                         begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());

                validEdge->push_back(curr_frame.edge_world->points[i]);
            }
//            if(edge_map.NeighborSearch(curr_frame.edge_world->points[i],1,neighbor)){
//
//                //opt_edge_num++;
//                std::vector<Eigen::Vector3d> nearCorners;
//                Eigen::Vector3d center(0, 0, 0);
//                for(int i=0;i<neighbor.points.size();i++){
//                    Eigen::Vector3d tmp(neighbor.points[i].x,neighbor.points[i].y,neighbor.points[i].z);
//                    center =center+tmp;
//                    nearCorners.push_back(tmp);
//                }
//
//                center=center/5.0;
//                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
//                for (int j = 0; j < 5; j++)
//                {
//                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
//                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
//                }
//
//                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
//                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
//                Eigen::Vector3d curr_point(raw_point.x, raw_point.y, raw_point.z);
//                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
//                    opt_edge_num++;
//                    //取两个对应角点a,b
//                    Eigen::Vector3d point_on_line = center;
//                    Eigen::Vector3d point_a, point_b;
//                    point_a = 0.1 * unit_direction + point_on_line;
//                    point_b = -0.1 * unit_direction + point_on_line;
//
//                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0,alpha);
//
//                    problem.AddResidualBlock(cost_function, loss_function,
//                                             begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());
//                    validEdge->push_back(curr_frame.edge_world->points[i]);
//                }
//            }
        }
       // pcl::copyPointCloud(*validEdge,*curr_frame.edgeDS);
    }

    void AddPlaneProblem(ceres::Problem &problem,ceres::LossFunction *loss_function ,Eigen::Quaterniond &begin_quat, Eigen::Quaterniond &end_quat, Eigen::Vector3d & begin_trans, Eigen::Vector3d & end_trans){
        validPlane->clear();
        for(int i=0;i<curr_frame.planeDS->size();i++){
            PointXYZIRT raw_point=curr_frame.planeDS->points[i];
            double alpha = (raw_point.timestamp -timeStart)/(timeEnd -timeStart);

            // Eigen::Vector3d  sensor_location = (1- alpha) * begin_trans + alpha * end_trans;
            VoxelBlock<PointXYZIRT> neighbor;
            Eigen::Vector4d pabcd;
            Eigen::Vector3d curr_point(raw_point.x,raw_point.y,raw_point.z);
            if(plane_map.NeighborSearchEstiPlane(curr_frame.plane_world->points[i],0.8,pabcd)){
                opt_plane_num++;
                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor, 1, 4, 4, 3, 3>(
                        new CTFunctor(alpha, curr_point, pabcd, 1.0));
                problem.AddResidualBlock(cost_function,
                                         loss_function,
                                         begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());

                validPlane->push_back(curr_frame.plane_world->points[i]);
            }

//            if( plane_map.NeighborSearch(curr_frame.plane_world->points[i],1,neighbor)){
//                bool planeValid=true;
//                Eigen::Matrix<double, 5, 3> matA0;
//                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
//                for (int j = 0; j < 5; j++)
//                {
//                    matA0(j, 0) = neighbor.points[j].x;
//                    matA0(j, 1) = neighbor.points[j].y;
//                    matA0(j, 2) = neighbor.points[j].z;
//                    //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
//                }
//                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
//                double negative_OA_dot_norm = 1 / norm.norm();
//                norm.normalize();
//                for (int j = 0; j < 5; j++)
//                {
//                    // if OX * n > 0.2, then plane is not fit well
//                    //计算每个点到最小二乘拟合平面的距离，距离过大则平面不够平
//                    if (fabs(norm(0) * neighbor.points[j].x +
//                             norm(1) * neighbor.points[j].y +
//                             norm(2) * neighbor.points[j].z + negative_OA_dot_norm) > 0.2)
//                    {
//                        //std::cout<<"plan not flat"<<std::endl;
//                        planeValid = false;
//                        break;
//                    }
//                }
//                if(planeValid){
//                    validPlane->push_back(raw_point);
//                    Eigen::Vector3d curr_point(raw_point.x, raw_point.y, raw_point.z);
//                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm, alpha);
//                    problem.AddResidualBlock(cost_function, loss_function,
//                                             begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());
//                    opt_plane_num++;
//                    validPlane->push_back(curr_frame.plane_world->points[i]);
//                }
//
//            }

        }

       // pcl::copyPointCloud(*validPlane,*curr_frame.planeDS);
    }

    void preprocess()
    {
        ros::Rate r(100);
        while (ros::ok())
        {

            if (edgeCloudQue.empty() || planesCloudQue.empty() || fullCloudQue.empty() || odoInitQue.empty())
            {
                continue;
            }

            ROS_INFO("frame_id:%d",frame_count);

            // get pcl cloud

            //cloud_full=std::move(fullCloudQue.front());

            headertime = fullCloudQue.front().header.stamp.toSec();

            while(!edgeCloudQue.empty() && edgeCloudQue.front().header.stamp.toSec()<headertime){
                edgeCloudQue.pop_front();
            }
            while(!planesCloudQue.empty() && planesCloudQue.front().header.stamp.toSec()<headertime){
                planesCloudQue.pop_front();
            }
            while(!odoInitQue.empty()&& odoInitQue.front().header.stamp.toSec()<headertime){
                odoInitQue.pop_front();
            }
            if(edgeCloudQue.empty() || headertime != edgeCloudQue.front().header.stamp.toSec() ||
                planesCloudQue.empty() || headertime != planesCloudQue.front().header.stamp.toSec()||
                odoInitQue.empty()||headertime !=odoInitQue.front().header.stamp.toSec() ){
                ROS_ERROR("time wrong!");
                continue;
            }

            frame_count++;

            cloud_full=fullCloudQue.front();
            cloud_edge = edgeCloudQue.front();
            cloud_plane =planesCloudQue.front();


            double w,x,y,z;
            w=odoInitQue.front().pose.pose.orientation.w;
            x=odoInitQue.front().pose.pose.orientation.x;
            y=odoInitQue.front().pose.pose.orientation.y;
            z=odoInitQue.front().pose.pose.orientation.z;
            Eigen::Quaterniond curr2last_rot(w,x,y,z);
//            Eigen::Quaterniond curr2last_rot(odoInitQue.front().pose.pose.orientation.w,  odoInitQue.front().pose.pose.orientation.x,
//                                               odoInitQue.front().pose.pose.orientation.y,  odoInitQue.front().pose.pose.orientation.z);
            Eigen::Vector3d  curr2last_trans(odoInitQue.front().pose.pose.position.x, odoInitQue.front().pose.pose.position.y, odoInitQue.front().pose.pose.position.z);
//            Eigen::Vector3d curr2last_trans(0,0,0);
            last2curr = SE3(curr2last_rot,curr2last_trans);

            ROS_INFO("TIME : %f, %f,%f,%f", cloud_full.header.stamp.toSec(),cloud_edge.header.stamp.toSec(),
                     cloud_plane.header.stamp.toSec(),odoInitQue.front().header.stamp.toSec());

            laserOdometryTrans.stamp_ = ros::Time(headertime);
            laserOdometryTrans.setRotation(tf::Quaternion(x, y, z, w));
            laserOdometryTrans.setOrigin(tf::Vector3(curr2last_trans.x(),curr2last_trans.y(),curr2last_trans.z()));

            edgeCloudQue.pop_front();
            planesCloudQue.pop_front();
            fullCloudQue.pop_front();
            odoInitQue.pop_front();

            pcl::moveFromROSMsg(cloud_plane,*plane);
            pcl::moveFromROSMsg(cloud_edge,*edge);
            pcl::moveFromROSMsg(cloud_full,*cloud_ori);

            if (first_flag )
            {
                // do some initial work

                //curr_frame.setFrameTime(headertime, timeStart, timeEnd, frame_count);
                //curr_frame.setOricloud(cloud_ori);
                ROS_INFO("init");
                curr_frame.pose.initialMotion();



                edge_map.InsertPointCloud(edge, curr_frame.pose);
                plane_map.InsertPointCloud(plane,curr_frame.pose);

                *features =*edge + *plane;
                *last_features = * features;



                poses.push_back(curr_frame.pose);

                first_flag = false;

                *edgeGlobalMap += *edge;
                *planeGlobalMap += *plane;
                *globalMap = *edgeGlobalMap + * planeGlobalMap;

                pcl::copyPointCloud(*plane, *lastPlane);
                pcl::copyPointCloud(*edge, *lastEdge);

                plane->clear();
                edge->clear();



                publish();
                continue;
            }

            auto initframe_start = std::chrono::steady_clock::now();
            //std::vector<int> indices;
            //pcl::removeNaNFromPointCloud(*cloud_ori, *cloud_ori, indices);
            std::sort(cloud_ori->points.begin(),cloud_ori->points.end(), timelist);
            timeStart = cloud_ori->points[0].timestamp;
            timeEnd = cloud_ori->points.back().timestamp;

            // init frame_info

            Initframe();
            auto initframe_end = std::chrono::steady_clock::now();

            if(debug_print) {
                ROS_INFO("INIT COST: %f ms", std::chrono::duration<double, std::milli>(initframe_end-initframe_start).count());
            }


            for (int iter_count = 0; iter_count < iter_nums; iter_count++) {
                //neighbor.clear();//size=0
                // find correspance

                // ceres opt
                ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.2);
                ceres::Problem problem;

                Eigen::Quaterniond  begin_quat = curr_frame.beginQuat();
                Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans();
                Eigen::Quaterniond end_quat = curr_frame.endQaut();
                Eigen::Vector3d end_trans = curr_frame.getEndTrans();

                double *begin_quat_param = curr_frame.beginQuat().coeffs().data();
                double *end_quat_param = curr_frame.endQaut().coeffs().data();
                double *begin_trans_param = curr_frame.getBeginTrans().data();
                double *end_trans_param = curr_frame.getEndTrans().data();

                problem.AddParameterBlock(begin_quat.coeffs().data(),4, new ceres::EigenQuaternionParameterization());
                problem.AddParameterBlock(end_quat.coeffs().data(),4, new ceres::EigenQuaternionParameterization());

                problem.AddParameterBlock(begin_trans.data(), 3);

                problem.AddParameterBlock(end_trans.data(), 3);


                auto findNeighbor_start = std::chrono::steady_clock::now();
                int opt_num=0;

                opt_edge_num=0;
                opt_plane_num=0;
                cloud_valid->clear();

                curr_frame.updateFeature();
                AddPlaneProblem(problem,loss_function,begin_quat,end_quat,begin_trans,end_trans);
                AddEdgeProblem(problem,loss_function, begin_quat,end_quat,begin_trans,end_trans);



                if(debug_print){
                    ROS_INFO("Edge_validnum: %d, plane_validnum: %d",opt_edge_num,opt_plane_num);
                }

                *cloud_valid = *curr_frame.edge_world+*curr_frame.plane_world;
                tfBroadcaster.sendTransform(laserOdometryTrans);
                publishCloud(cloud_valid_pub,cloud_valid,ros::Time(headertime),"map");
                auto findNeighbor_end = std::chrono::steady_clock::now();

                if (debug_print) {
                    ROS_INFO("cloud find neighbor COST: %f ms",
                             std::chrono::duration<double, std::milli>(findNeighbor_end - findNeighbor_start).count());
                }
                //add other constraints
//                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistency,3,3>(
//                                                 new LocationConsistency(poses.back().endTrans(),std::sqrt((opt_edge_num+opt_plane_num)*0.001))),
//                                         nullptr,
//                                         begin_trans.data()
//
//                );
//                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocity,3,3,3>(
//                                                 new ConstantVelocity(poses.back().endTrans()-poses.back().beginTrans(),std::sqrt((opt_edge_num+opt_plane_num)*0.001))),
//                                         nullptr,
//                                         begin_trans.data(),end_trans.data()
//                );
                // solve
                ceres::Solver::Options option;
                option.linear_solver_type = ceres::DENSE_QR;
                option.max_num_iterations =5;
                option.num_threads = 3;
                option.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

                ceres::Solver::Summary summary;
                auto solve_start = std::chrono::steady_clock::now();
                ceres::Solve(option, &problem, &summary);

                auto solve_end = std::chrono::steady_clock::now();
                if (debug_print) {
                    ROS_INFO("SLOVE COST: %f ms",
                             std::chrono::duration<double, std::milli>(solve_end - solve_start).count());

//                    std::cout<<begin_trans<<"&&"<<end_trans<<endl;
                }

                begin_quat.normalize();
                end_quat.normalize();
                curr_frame.setMotion(begin_quat, end_quat, begin_trans, end_trans);
                curr_frame.updateFeature();
                //curr_frame.normalize();

                //curr_frame.update();


                if(curr_frame.pose.compareDiff(poses.back())) {
                    poses.push_back(curr_frame.pose);
                    break;
                }




            }
//            EulerAngles end_euler = ToEulerAngles(pre_pose.endQuat());
//            ROS_INFO("ruler: %f, %f,%f", end_euler.roll, end_euler.pitch, end_euler.yaw);

            // update local_map
            auto map_start = std::chrono::steady_clock::now();
            edge_map.RemoveFarFromLocation(curr_frame.getEndTrans(),300);
            edge_map.InsertPointCloud(curr_frame.edge_world,curr_frame.pose);

            plane_map.RemoveFarFromLocation(curr_frame.getEndTrans(),300);
            plane_map.InsertPointCloud(curr_frame.plane_world,curr_frame.pose);


//            feature_map.InsertPointCloud(curr_frame.plane_world,curr_frame.pose);
//            feature_map.InsertPointCloud(curr_frame.edge_world,curr_frame.pose);

            poses.push_back(curr_frame.pose);
            auto map_end = std::chrono::steady_clock::now();
            ROS_INFO("MAP UPDATE COST: %f ms",std::chrono::duration<double,std::milli>(map_end-map_start).count());
            //updateLocalMap(curr_frame,neighbor);
//            ROS_INFO("MAP SIZE: %d", local_map.size());


            *edgeGlobalMap += *curr_frame.edge_world;
            *planeGlobalMap += *curr_frame.plane_world;
            *globalMap = *edgeGlobalMap + * planeGlobalMap;

            *last_features= *features;
            pcl::copyPointCloud(*curr_frame.plane_world, *lastPlane);
            pcl::copyPointCloud(*curr_frame.edge_world, *lastEdge);
            plane->clear();
            edge->clear();


            // publish map
            publish();
            // debugPrint();
            r.sleep();
        }
    }


    void publishCloud(ros::Publisher &pub, pcl::PointCloud<PointXYZIRT>::ConstPtr pcl_cloud, ros::Time header_stamp ,string frame){
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_cloud, ros_cloud);
        ros_cloud.header.stamp = header_stamp;
        ros_cloud.header.frame_id = frame;
        pub.publish(ros_cloud);
        return ;
    }

    void publish(){
        nav_msgs::Odometry odometry_pose;
        odometry_pose.header.stamp = ros::Time(headertime);
        odometry_pose.header.frame_id = odometry_frame;
        odometry_pose.pose.pose.orientation.w = curr_frame.endQaut().w();
        odometry_pose.pose.pose.orientation.x = curr_frame.endQaut().x();
        odometry_pose.pose.pose.orientation.y = curr_frame.endQaut().y();
        odometry_pose.pose.pose.orientation.z = curr_frame.endQaut().z();

        odometry_pose.pose.pose.position.x = curr_frame.getEndTrans().x();
        odometry_pose.pose.pose.position.y = curr_frame.getEndTrans().y();
        odometry_pose.pose.pose.position.z = curr_frame.getEndTrans().z();


        sensor_msgs::PointCloud2 globalMapRos;
        pcl::toROSMsg(*globalMap, globalMapRos);
        globalMapRos.header.stamp = ros::Time(headertime);
        globalMapRos.header.frame_id = "map";

        geometry_msgs::PoseStamped geo_odometry_pose;
        geo_odometry_pose.header = odometry_pose.header;
        geo_odometry_pose.pose = odometry_pose.pose.pose;

        globalPath.header.stamp = ros::Time(headertime);
        globalPath.header.frame_id = "map";
        globalPath.poses.push_back(geo_odometry_pose);

        sensor_msgs::PointCloud2 deskew_cloud;
        pcl::toROSMsg(*curr_frame.cloud_deskew,deskew_cloud);
        deskew_cloud.header.stamp = ros::Time(headertime);
        deskew_cloud.header.frame_id= "odometry";

        sensor_msgs::PointCloud2 world_cloud;
        pcl::toROSMsg(*curr_frame.cloud_world,world_cloud);
        world_cloud.header.stamp = ros::Time(headertime);
        world_cloud.header.frame_id="map";


        sensor_msgs::PointCloud2 ori_cloud;
        pcl::toROSMsg(*curr_frame.cloud_ori,ori_cloud);
        ori_cloud.header.stamp = ros::Time(headertime);
        ori_cloud.header.frame_id= "odometry";

        cloud_ori_pub.publish(ori_cloud);
        cloud_pub.publish(deskew_cloud);
        cloud_world_pub.publish(world_cloud);

        traj_pub.publish(globalPath);

        odo_pub.publish(odometry_pose);



        map_pub.publish(globalMapRos);


    }

    void debugPrint()
    {
        if (debug_print)
        {
            ROS_INFO("Before remove Nan, size: %d", cloud_ori->size());
            ROS_INFO("After remove Nan, size: %d", cloud_ori->size());
        }
    }
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "adact_odometry");
    ROS_INFO("START LIDAR ODOMETRY!");

//
    Lidar_odo lp;
//
//    // // 由于process是循环，如果直接运行，不会进入下面的spin，就不会进入点云回调函数，所以要新开一个线程
    std::thread process(&Lidar_odo::preprocess, &lp);
//
//    // // spin才会进入回调函数
    ros::spin();
//
//    // // 加入线程
    process.join();

    return 0;
}