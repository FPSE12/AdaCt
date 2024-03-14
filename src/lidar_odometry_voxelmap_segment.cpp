#include "AdaCt/utility.h"
#include "AdaCt/frame_info_segment.hpp"

//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;

#define MAX_ITER_COUNT 5
#define MAX_NEIGHBOR_NUM 20

#define A2D_THRESHOLD 0.4 //UNSTRUCTE:low; , structure: >0.4



//



class Lidar_odo : public configParam
{
public:
    ros::Subscriber lidar_sub;

    ros::Publisher map_pub;
    ros::Publisher odo_pub;
    ros::Publisher traj_pub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_ori_pub;
    ros::Publisher cloud_valid_pub;
    ros::Publisher cloud_world_pub;
    ros::Publisher cloud_downsample_pub;


    // cloud
    std::deque<sensor_msgs::PointCloud2> rosCloudQue;
    sensor_msgs::PointCloud2 cloud_ori_ros;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointVelodyne>::Ptr cloud_vel;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;

    pcl::PointCloud<PointXYZIRT>::Ptr cloud_last;

    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

    double timeStart, timeEnd, headertime;
    frame_info curr_frame;

    std::mutex mtx;

    bool first_flag;
    int frame_count;

    // debug info
    int beforeRemoveNan, afterRomoveNan;

    // map
    Voxelmap<PointXYZIRT> local_map;
    // opt pose
//    OptPose pre_pose, curr_pose;

    std::vector<OptPose> key_poses;
    OptPose last_pose;
    int iter_nums; // 5

    int motion_evaluate;

    double a2d_average;



    Lidar_odo()
    {
        //ROS_INFO("1");
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_odo::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/targetcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);
        cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/world_points",1);
        cloud_downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/downsample",1);

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_vel.reset(new pcl::PointCloud<PointVelodyne>());
        cloud_last.reset(new pcl::PointCloud<PointXYZIRT>());


        first_flag = true;
        frame_count =-1;

        iter_nums =MAX_ITER_COUNT;


        rosCloudQue.clear();

//        pre_pose.initialMotion();
//        curr_pose.initialMotion();

        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());

//        last_pose.initialMotion();

        motion_evaluate=0;

    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
        //preprocess();
    }

    void changeCloudFormat(pcl::PointCloud<PointVelodyne>::Ptr vel, pcl::PointCloud<PointXYZIRT>::Ptr rs, double headertime){
        for(auto & point_vel : vel->points){
            PointXYZIRT point_rs;
            point_rs.x = point_vel.x;
            point_rs.y = point_vel.y;
            point_rs.z = point_vel.z;
            point_rs.ring = point_vel.ring;
            point_rs.intensity = point_vel.intensity;
            point_rs.timestamp =(double) point_vel.time + headertime;
//            ROS_INFO("TIMESTAMP: %f",point_rs.timestamp);
            rs->push_back(point_rs);
        }
    }

    void Initframe()
    {
        curr_frame.Reset();

        curr_frame.setOricloud(cloud_ori);
        if(frame_count<1){
            curr_frame.pose.initialMotion();
            //key_poses.push_back(curr_frame.pose);
        }else{

//            curr_frame.pose.begin_pose = key_poses[frame_count].begin_pose *(key_poses[frame_count-1].begin_pose.inverse() * key_poses[frame_count].begin_pose);
//            curr_frame.pose.end_pose = key_poses[frame_count].end_pose * (key_poses[frame_count-1].end_pose.inverse() * key_poses[frame_count].end_pose);
            //OptPose last_pose = key_poses.back();
            curr_frame.pose.begin_pose = key_poses.back().end_pose;
            curr_frame.pose.end_pose = curr_frame.pose.begin_pose *(key_poses.back().begin_pose.inverse() * key_poses.back().end_pose);
            //curr_frame.pose.end_pose = key_poses.back().end_pose *(key_poses[key_poses.size()-2].end_pose.inverse() * key_poses.back().end_pose);
        }
        curr_frame.getTimeStamp(headertime,timeStart,timeEnd);
        //curr_frame.Segment();
        curr_frame.findRingStartEnd();
        publishCloud(cloud_pub,curr_frame.segment_cloud,ros::Time(headertime),"odometry");
//        curr_frame.grid_sample_mid(DOWNSAMPLE_VOXEL_SIZE);

        curr_frame.Adaptive_sample_mid();
    }

    bool SearchNeighbor_(const Voxelmap<PointXYZIRT> &local_map, const Eigen::Vector3d & PointW,
                        double searchThreshold, VoxelBlock<PointXYZIRT> & neighbor_block, int max_neighbor_num=MAX_NEIGHBOR_NUM){
//        Eigen::Vector3d PointW_(PointW.x, PointW.y,PointW.z);

        Voxel voxel = Voxel::Coordinates(PointW,MAP_VOXEL_SIZE);
        int kx=voxel.x;
        int ky=voxel.y;
        int kz=voxel.z;

        Neighbors_queue neighborsQueue;

        for(int kxx=kx-1;kxx<kx+1+1;++kxx){
            for(int kyy=ky-1;kyy<ky+1+1;++kyy){
                for(int kzz=kz-1;kzz<kz+1+1;++kzz){
                    voxel.x=kxx;
                    voxel.y=kyy;
                    voxel.z=kzz;

                    auto search = local_map.map.find(voxel);
                    if(search != local_map.map.end()){
                        auto & voxel_block = search.value();

                        for(auto point : voxel_block.points){
                            Eigen::Vector3d neighbor(point.x,point.y,point.z);

                            double dis=(neighbor[0]-PointW[0])*(neighbor[0]-PointW[0])+(neighbor[1]-PointW[1])*(neighbor[1]-PointW[1])
                                    +(neighbor[2]-PointW[2])*(neighbor[2]-PointW[2]);
                            //ROS_INFO("DIS:%f,",dis);
//                            if(dis<searchThreshold){
                                if(neighborsQueue.size() == MAX_NEIGHBOR_NUM){
                                    if(dis < std::get<0>(neighborsQueue.top())){
                                        neighborsQueue.pop();

                                        neighborsQueue.emplace(dis,neighbor,voxel);


                                    }
                                }else{

                                    neighborsQueue.emplace(dis,neighbor,voxel);

                                }
//                            }
                        }
                    }

                }
            }
        }

        if(neighborsQueue.size() >= MAX_NEIGHBOR_NUM ){//estiplane?

            while(!neighborsQueue.empty()){
//                 Eigen::Vector3d temp;
//                 temp[0]=std::get<1>(neighborsQueue.top()).x();
//                 temp[1]=std::get<1>(neighborsQueue.top()).y();
//                 temp[2]=std::get<1>(neighborsQueue.top()).z();
                //neighbor.push_back(temp);

                PointXYZIRT temp;
                temp.x=std::get<1>(neighborsQueue.top()).x();
                temp.y=std::get<1>(neighborsQueue.top()).y();
                temp.z=std::get<1>(neighborsQueue.top()).z();
                neighbor_block.points.push_back(temp);
                neighborsQueue.pop();
            }
//                 neighbor.addPoint(std::get<1>(neighborsQueue.top()));

            return true;
        }

        return false;

    }


    void computeDistribution(VoxelBlock<PointXYZIRT> & neighbor, double & a2D, Eigen::Vector3d & normal){
//        bool isValid = true;
//        if(neighbor.points.size()<5){
//            isValid= false;
//            return ;
//        }
        Eigen::Vector3d braycenter = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

        Eigen::Vector3d point_ref;

        for(auto &point:neighbor.points){
            point_ref = getPointXYZ(point);
            braycenter += point_ref;
            //cov += (point_ref*point_ref.transpose());
        }

        braycenter /= (double) neighbor.points.size();

        //faster
        for(auto &point : neighbor.points){
            Eigen::Vector3d temp_point = getPointXYZ(point);
            for(int k=0;k<3;k++){
                for(int l=k;l<3;l++){
                    cov(k,l)  += (temp_point(k)-braycenter(k)) *(temp_point(l)-braycenter(l));
                }
            }
        }
        cov(1,0)=cov(0,1);
        cov(2,0)=cov(0,2);
        cov(2,1)=cov(1,2);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d > es(cov);


        normal = es.eigenvectors().col(0).normalized();


        double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
        double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
        double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));


        a2D = (sigma_2 - sigma_3)/sigma_1;

        //return isValid;
    }


    void getGlobalMap(const Voxelmap<PointXYZIRT> &local_map,pcl::PointCloud<PointXYZIRT>::Ptr all_point){
        all_point->clear();
        for(auto &voxel : local_map.map){
            for(auto &point : voxel.second.points){
                all_point->points.push_back(point);
            }
        }
    }



    void preprocess()
    {
        ros::Rate r(100);//100Hz

        while (ros::ok())
        {

            if (rosCloudQue.size() == 0)
            {
                continue;
               // return;
            }

            auto all_start = std::chrono::steady_clock::now();

            frame_count++;
            ROS_INFO("------------------------------frame_id:%d",frame_count);

            // get pcl cloud
            motion_evaluate =0;
            cloud_ori_ros = std::move(rosCloudQue.front());
            rosCloudQue.pop_front();

            headertime = cloud_ori_ros.header.stamp.toSec();

            cloud_ori->clear();
            cloud_vel->clear();

            if(lidar_type == VLP){
                //ROS_INFO("VLP");
                pcl::moveFromROSMsg(cloud_ori_ros,*cloud_vel);
                changeCloudFormat(cloud_vel,cloud_ori,headertime);

            }else{
                pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);
            }

//           ---------------------------------------------------init------------------------------------
            auto initframe_start = std::chrono::steady_clock::now();

            if (first_flag )
            {
                // do some initial work
                //ROS_INFO("1");

                curr_frame.pose.initialMotion();

                local_map.InsertPointCloud(cloud_ori);

//                last_pose=curr_frame.pose;
                key_poses.push_back(curr_frame.pose);
                //ROS_INFO("2");
                first_flag = false;

                *globalMap += *cloud_ori;

                publish();
                //ROS_INFO("3");
                continue;
                //return;
            }




            // init frame_info

            last_pose = key_poses.back();

            Initframe();
            //cloud_ori->clear();
            auto initframe_end = std::chrono::steady_clock::now();
            double init_cost = std::chrono::duration<double, std::milli>(initframe_end-initframe_start).count();
            if(debug_print) {
                ROS_INFO("After DownSample: %d",curr_frame.cloud_ori_downsample->size());
                ROS_INFO("INIT COST: %f ms", init_cost);
            }


//            ---------------------------------- solve--------------------------------------------------
            int iter_count = 0;
            double neighbor_find_average=0;
            double solve_average=0;


//            vector<double> loss={0.2,0.2,0.2,0.2,0.2};

            for (; iter_count < iter_nums; iter_count++) {
                //neighbor.clear();//size=0
                // find correspance

//                if( motion_evaluate==2){
//                    ROS_WARN("BIG MOTION! LOW DOWNSAMPLE!!");
//                    motion_evaluate =0;
//                    curr_frame.grid_sample_mid(0.5 * DOWNSAMPLE_VOXEL_SIZE);
//                }

                // ceres opt
                ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.2);
                //ceres::LossFunction * loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem problem;

                Eigen::Quaterniond  begin_quat = curr_frame.beginQuat();
                Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans();
                Eigen::Quaterniond end_quat = curr_frame.endQaut();
                Eigen::Vector3d end_trans = curr_frame.getEndTrans();

                double *begin_quat_param = curr_frame.beginQuat().coeffs().data();
                double *end_quat_param = curr_frame.endQaut().coeffs().data();
                double *begin_trans_param = curr_frame.getBeginTrans().data();
                double *end_trans_param = curr_frame.getEndTrans().data();

//                problem.AddParameterBlock(begin_quat.coeffs().data(),4);
//                problem.SetManifold(begin_quat.coeffs().data(), new ceres::EigenQuaternionManifold);
//
//                problem.AddParameterBlock(end_quat.coeffs().data(), 4);
//                problem.SetManifold(end_quat.coeffs().data(),new ceres::EigenQuaternionManifold);


                problem.AddParameterBlock(begin_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold);
                problem.AddParameterBlock(end_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold);

                problem.AddParameterBlock(begin_trans.data(), 3);

                problem.AddParameterBlock(end_trans.data(), 3);

                // add parameterblock
                // auto & begin_pose=curr_frame.pose.begin_pose;
                // auto & end_pose=curr_frame.pose.end_pose;

                // problem.AddParameterBlock(begin_pose.data(),Sophus::SE3d::num_parameters, new LocalParameterizationSE3);
                // problem.AddParameterBlock(end_pose.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3);

                int opt_num=0;
                auto findNeighbor_start = std::chrono::steady_clock::now();
                double find_neighborcost=0;
                double compouteDisatribute=0;

                cloud_valid->clear();
//#pragma omp parallel for num_threads(8)

                for (auto point : curr_frame.cloud_ori_downsample->points) {


                    //Neighbors_queue neighborsQueue;
                    VoxelBlock<PointXYZIRT> neighbor;
//                    std::vector<Eigen::Vector3d > neighbor;

                    //neighbor.clear();

                    double searchDis = 0.8;

                    Eigen::Vector4d pabcd;

                    double alpha = (point.timestamp - timeStart)/( timeEnd-timeStart);
                    Eigen::Vector3d raw_point(point.x,point.y,point.z);
                    Eigen::Vector3d world_point;
                    SE3 inter_pose=curr_frame.pose.linearInplote(alpha);
                    world_point = inter_pose * raw_point;


                    auto time1 = std::chrono::steady_clock::now();
                    //bool IsValid=local_map.NeighborSearch(curr_frame.cloud_world->points[i],raw_point,searchDis,neighbor,pabcd);
                    bool IsValid= SearchNeighbor_(local_map, world_point,searchDis,neighbor);
                    auto time2 = std::chrono::steady_clock::now();
                    find_neighborcost += std::chrono::duration<double, std::milli>(time2 - time1).count();



                    if(IsValid){


                        auto time3 = std::chrono::steady_clock::now();
//                        neighbor.computeDescription(A2D | NORMAL);
                        double a2d;
                        Eigen::Vector3d normal;
                        computeDistribution(neighbor,a2d,normal);
                        if(normal.dot(begin_trans-raw_point)<0){
                            normal = -1.0 * normal;
                        }
//                        VoxelBlockDescription<double> description= estimateNeighborHood(neighbor, raw_point, begin_trans);
                        auto time4  = std::chrono::steady_clock::now();

                        compouteDisatribute += std::chrono::duration<double, std::milli>(time4 - time3).count();
//                        double weight=0.9*a2d*a2d+
//                                0.1*std::exp(-(getPointXYZ(neighbor.points.back())- world_point).norm()/(0.3*20));
//                        double weight = a2d<0.5?0:a2d;//remove no plane

                        if(a2d<A2D_THRESHOLD) continue;
                        double weight = a2d;
                        opt_num++;
//                        double dis_point = std::abs((world_point-getPointXYZ(neighbor.points.back())).transpose() * normal) ;
//                        if(dis_point<1){// reduce the num opt



//                         ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor, 1, 4, 4, 3, 3>(
//                                new CTFunctor(alpha, raw_point, pabcd, weight));
//                            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor2, 1, 4, 4, 3, 3>(
//                                    new CTFunctor2(alpha, raw_point, getPointXYZ(neighbor.points.back()), weight,neighbor.description.normal));
                            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor2, 1, 4, 4, 3, 3>(
                                new CTFunctor2(alpha, raw_point, getPointXYZ(neighbor.points.back()), weight,normal));

                            problem.AddResidualBlock(cost_function,
                                                     loss_function,
                                                     begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());
                            cloud_valid->push_back(point);
//                        }


                    }
                }
                publishCloud(cloud_valid_pub,cloud_valid,ros::Time(headertime),"odometry");//1ms
//
                publishCloud(cloud_downsample_pub,curr_frame.cloud_ori_downsample,ros::Time(headertime),"odometry");
                auto findNeighbor_end = std::chrono::steady_clock::now();

                neighbor_find_average = std::chrono::duration<double, std::milli >(findNeighbor_end-findNeighbor_start).count();

                if (debug_print) {
//                    ROS_INFO("neighbor COST: %f ms",
//                             find_neighborcost);
//                    ROS_INFO("compute COST: %f ms",
//                             compouteDisatribute);
//                    ROS_INFO("problem make COST: %f ms",
//                             std::chrono::duration<double, std::milli>(findNeighbor_end - findNeighbor_start).count());
//                    ROS_INFO("OPT_NUM:%d",opt_num);


                }

                //add other constraints
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistency,6,4,3>(
                        new LocationConsistency(key_poses.back().endQuat(), key_poses.back().endTrans(), std::sqrt(opt_num * 0.01))),
                                         nullptr,
                                         begin_quat.coeffs().data(),begin_trans.data()

                        );
//                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocity,3,3,3>(
//                        new ConstantVelocity(key_poses.back().endTrans()-key_poses.back().beginTrans(),std::sqrt(opt_num*0.001))),
//                        nullptr,
//                        begin_trans.data(),end_trans.data()
//                        );

                //add disconnecty
//                int disconnecty = curr_frame.first_line.size();
//                ROS_INFO("disconnecty:%d",disconnecty);
//                for(int i=0;i<disconnecty;i++){
//                    double alpha1 = (curr_frame.timestamp_line[i].first - timeStart) / (timeEnd - timeStart);
//                    double alpha2 = (curr_frame.timestamp_line[i].second - timeStart) / (timeEnd - timeStart);
////                    ROS_INFO("ALPHA: %f, %f",alpha1,alpha2);
//                    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Disconnecty,1,4,4>(
//                            new Disconnecty(curr_frame.first_line[i],curr_frame.last_line[i],alpha1,alpha2,disconnecty * 0.01)),
//                            nullptr,
//                            begin_quat.coeffs().data(),end_quat.coeffs().data()
//                    );
//                }

                Eigen::Quaterniond pre_quat_delta = key_poses.back().beginQuat().conjugate() * key_poses.back().endQuat();
                pre_quat_delta.normalize();
//                Eigen::Vector3d pre_trans_delta = key_poses.back().beginQuat().conjugate()* key_poses.back().endTrans()-
//                        key_poses.back().beginQuat().conjugate()*key_poses.back().beginTrans();
                Eigen::Vector3d pre_trans_delta = key_poses.back().endTrans() - key_poses.back().beginTrans();//in world axis

//                SE3  delta_pose = last_pose.begin_pose.inverse() * last_pose.end_pose;
//                Eigen::Quaterniond pre_quat_delta = delta_pose.unit_quaternion();
//                Eigen::Vector3d pre_trans_delta = delta_pose.translation();

                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocityRotTran,6,4,4,3,3>(
                        new ConstantVelocityRotTran(pre_quat_delta,pre_trans_delta,std::sqrt(opt_num*0.001))),
                                         nullptr,
                        begin_quat.coeffs().data(),end_quat.coeffs().data(),begin_trans.data(),end_trans.data()
                        );


                // solve
                ceres::Solver::Options option;
                option.linear_solver_type = ceres::DENSE_QR;
                option.max_num_iterations =6;
                option.num_threads = 16;
                option.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

                ceres::Solver::Summary summary;
                auto solve_start = std::chrono::steady_clock::now();
                ceres::Solve(option, &problem, &summary);

                auto solve_end = std::chrono::steady_clock::now();

                solve_average = std::chrono::duration<double, std::milli >(solve_end-solve_start).count();

//                if (debug_print) {
//                    ROS_INFO("SLOVE COST: %f ms",
//                             std::chrono::duration<double, std::milli>(solve_end - solve_start).count());
//
////                    std::cout<<begin_trans<<"&&"<<end_trans<<endl;
//                }

                begin_quat.normalize();
                end_quat.normalize();
                curr_frame.setMotion(begin_quat, end_quat, begin_trans, end_trans);
                //curr_frame.updateFromDownSample();
                //curr_frame.normalize();

                //curr_frame.update();


                motion_evaluate = curr_frame.pose.compareDiff(key_poses.back());

                if(motion_evaluate ==0){
                    break;
                }
            }

            ROS_INFO("finish with %d iters.", iter_count);
            ROS_INFO("find neighbor cost : %f",neighbor_find_average);
            ROS_INFO("solve : %f", solve_average);
//          --------------------------------------update local_map---------------------------------------
            auto map_start = std::chrono::steady_clock::now();
//            curr_frame.updateWorldCloud();
//            pcl::copyPointCloud(*cloud_valid,*curr_frame.cloud_ori_downsample );
            curr_frame.updateFromDownSample();
            local_map.RemoveFarFromLocation(curr_frame.getEndTrans(),100);
            local_map.InsertPointCloud(curr_frame.cloud_world);

            key_poses.push_back(curr_frame.pose);
            auto map_end = std::chrono::steady_clock::now();
            double map_cost = std::chrono::duration<double,std::milli>(map_end-map_start).count();
            ROS_INFO("MAP UPDATE COST: %f ms",map_cost);

            auto all_end = std::chrono::steady_clock::now();
            ROS_INFO("ALL COST: %f ms",std::chrono::duration<double, std::milli>(all_end-all_start).count());
            //updateLocalMap(curr_frame,neighbor);
//            ROS_INFO("MAP SIZE: %d", local_map.size());
//            *globalMap += *curr_frame.cloud_world;
            getGlobalMap(local_map, globalMap);

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



        sensor_msgs::PointCloud2 world_cloud;
        pcl::toROSMsg(*curr_frame.cloud_world,world_cloud);
        world_cloud.header.stamp = ros::Time(headertime);
        world_cloud.header.frame_id="map";


        sensor_msgs::PointCloud2 ori_cloud;
        pcl::toROSMsg(*curr_frame.cloud_ori,ori_cloud);
        ori_cloud.header.stamp = ros::Time(headertime);
        ori_cloud.header.frame_id= "odometry";

        cloud_ori_pub.publish(ori_cloud);

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
    ROS_INFO("START LIDAR PREPROCESS!");

//
    Lidar_odo lp;
//
//    // // 由于process是循环，如果直接运行，不会进入下面的spin，就不会进入点云回调函数，所以要新开一个线程
    std::thread process(&Lidar_odo::preprocess, &lp);
//
    //process.join();
//    // // spin才会进入回调函数
    ros::spin();
//
//    // // 加入线程
     process.join();
    Eigen::Quaterniond test;
    test.vec();
    return 0;
}