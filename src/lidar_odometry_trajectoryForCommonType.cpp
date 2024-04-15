#include "AdaCt/utility.h"
#include "AdaCt/frame_info_usecommonPointTypeOnly.hpp"
#include "AdaCt/cloud_convert.hpp"
#include "AdaCt/downSample.hpp"
#include "AdaCt/trajectory.hpp"
//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap_commontype.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;

#define MAX_ITER_COUNT 5
#define MAX_ITER_COUNT_SCAN 2

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


    //
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_world;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_deskew;
    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

    // double timeStart, timeEnd, headertime;
//    frame_info curr_frame;

    std::mutex mtx;
    std::mutex mtx_get;

    bool first_flag;
    int frame_count;

    // map
    VoxelmapCommon voxel_map;
    float max_voxel_size;
    int max_layer;
    vector<int> layer_size;
    int max_points_size;
    float min_eigen_value;

    std::vector<OptPose> key_poses;
    OptPose last_pose;
    std::vector<PointType> last_points;

    double trajectory_startTime;
    Trajectory globalTraj;


    int iter_nums; // 5

    //time cost ms
    double cost_threshold;
    double cost_robust;

    double a2d_average;


    Lidar_odo()
    {
        //ROS_INFO("1");
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_odo::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/deskew_pointcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);
        cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/world_points",1);
        cloud_downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/downsample",1);


        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());

        // Segcloud.reset(new pcl::PointCloud<PointXYZIRT>());

        rosCloudQue.clear();

        //time cost
        cost_threshold = 100.0;
        cost_robust = 30.0;


        // params init
        first_flag = true;
        frame_count =-1;
        iter_nums =MAX_ITER_COUNT;


        last_pose.initialMotion();
        //mapping params

        last_points.clear();

    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
        //preprocess();
    }

    void livoxLidarCallback(const livox_ros_driver::CustomMsgConstPtr &laserMsg){

    }



    void preprocess(pcl::PointCloud<PointXYZIRT>::Ptr pcl_cloud, std::vector<PointType> & point_vec,
                    double  &header_time, double &timeStart, double &timeEnd){

        for(auto & point_pcl: pcl_cloud->points){
            if(!std::isfinite(point_pcl.x)|| !std::isfinite(point_pcl.y) || !std::isfinite(point_pcl.z) ){
                continue;
            }
            double dis =point_pcl.x * point_pcl.x + point_pcl.y * point_pcl.y + point_pcl.z * point_pcl.z;
            if(dis<blind * blind){
                continue;
            }

            PointType p;
            p.point<<point_pcl.x, point_pcl.y, point_pcl.z;
            p.timestamp = point_pcl.timestamp;
            p.intensity = point_pcl.intensity;
            p.distance = dis;
            point_vec.emplace_back(p);
        }
        //do not calculate the alpha!
        sort(point_vec.begin(), point_vec.end(), timelistvec);
        timeStart = point_vec[0].timestamp;
        timeEnd = point_vec.back().timestamp;
        //sub frame like ct-icp
        subSampleFrame(point_vec,0.05);
    }

    //point2point
    //source_cloud is all last_frame in world axis
    void SCAN2SCAN(vector<PointType>& targetCloud, vector<PointType > &sourceCloud, OptPose &curr_pose_){

//        std::cout<<curr_pose_.end_pose.translation()<<std::endl;
        //time cost??
        tsl::robin_map<Voxel,vector<PointType>> grid;
        double downsample_size = 0.2;
        Voxel voxel;
        for(auto  s_point : sourceCloud ){
            voxel.x = static_cast<short>(s_point.point_world[0] / downsample_size);
            if(voxel.x<0) voxel.x--;
            voxel.y = static_cast<short>(s_point.point_world[1] / downsample_size);
            if(voxel.y<0) voxel.y--;
            voxel.z = static_cast<short>(s_point.point_world[2] / downsample_size);
            if(voxel.z<0) voxel.z--;
            grid[voxel].push_back(s_point);
        }
        for(int iter =0; iter<5;iter++){
            ceres::Problem problem;
            ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
            Eigen::Quaterniond  begin_quat_ = curr_pose_.beginQuat();
            Eigen::Vector3d  begin_trans_ = curr_pose_.beginTrans();
            Eigen::Quaterniond end_quat_ = curr_pose_.endQuat();
            Eigen::Vector3d end_trans_ = curr_pose_.endTrans();


            problem.AddParameterBlock(begin_quat_.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(end_quat_.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(begin_trans_.data(), 3);
            problem.AddParameterBlock(end_trans_.data(), 3);
            int valid_num=0;
            for(auto  t_point : targetCloud){

                t_point.point_world = curr_pose_.linearInplote(t_point.alpha)* t_point.point;

                voxel.x = static_cast<short>(t_point.point_world[0] / downsample_size);
                if(voxel.x<0) voxel.x--;
                voxel.y = static_cast<short>(t_point.point_world[1] / downsample_size);
                if(voxel.y<0) voxel.y--;
                voxel.z = static_cast<short>(t_point.point_world[2] / downsample_size);
                if(voxel.z<0) voxel.z--;

                if(grid.find(voxel)!=grid.end()){
                    double min_dis = std::numeric_limits<double>::max();
                    PointType target_point;
                    for(auto & s_point : grid[voxel]){
                        double dis = sqrt((s_point.point_world[0] - t_point.point_world[0])*(s_point.point_world[0] - t_point.point_world[0])
                                          +(s_point.point_world[1] - t_point.point_world[1])*(s_point.point_world[1] - t_point.point_world[1])
                                          +(s_point.point_world[2] - t_point.point_world[2])*(s_point.point_world[2] - t_point.point_world[2]));

                        if(dis<min_dis){
                            min_dis = dis;
                            target_point = s_point;
                        }
                    }

                    valid_num++;
                    double alpha = t_point.alpha;
                    double weight = exp(-min_dis);
                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<Point2pointFunctor, 1, 4, 4, 3, 3>(
                            new Point2pointFunctor(alpha, weight, target_point.point_world, t_point.point));

                    problem.AddResidualBlock(cost_function,
                                             loss_function,
                                             begin_quat_.coeffs().data(),end_quat_.coeffs().data(), begin_trans_.data(), end_trans_.data());
                }

            }

//            ROS_INFO("valid: %d",valid_num);
            ceres::Solver::Options option;
            option.linear_solver_type = ceres::DENSE_QR;
            option.max_num_iterations =6;
            option.num_threads = 16;
            option.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
            ceres::Solver::Summary summary;
            ceres::Solve(option, &problem, &summary);

            begin_quat_.normalize();
            end_quat_.normalize();
            curr_pose_.begin_pose=Sophus::SE3d(begin_quat_, begin_trans_);
            curr_pose_.end_pose=Sophus::SE3d(end_quat_, end_trans_);


        }
//        std::cout<<curr_pose_.end_pose.translation()<<std::endl;

    }

    void getSegCloud(std::vector<PointType>& cloud_ori,
                     double timeStart, double timeEnd,
                     double timeSegBegin, double timeSegEnd,
                     std::vector<PointType>& SegCloud){
        SegCloud.clear();
        if(timeSegBegin < timeStart || timeSegEnd > timeEnd || timeSegEnd < timeSegBegin){
            ROS_ERROR("cloud Seg false! timeSegBegin: %f, timeSegEnd: %f, timeStart: %f, timeEnd:%f ", timeSegBegin,
                      timeSegEnd,timeStart,timeEnd);
            ROS_BREAK();
        }
        for(PointType &point : cloud_ori){
            if(point.timestamp>= timeSegBegin && point.timestamp <= timeSegEnd){
                SegCloud.emplace_back(point);
            }
        }
    }

    void Initframe(frame_info & curr_frame,
                   std::vector<PointType> & points_vec,
                   Sophus::SE3d begin_pose, Sophus::SE3d end_pose,
                   double timeB, double timeE, double header_time,
                   int frame_id)
    {
        curr_frame.Reset();
        curr_frame.pose.begin_pose = begin_pose;
        curr_frame.pose.end_pose  = end_pose;
        curr_frame.timeStart = timeB;
        curr_frame.timeEnd = timeE;
        curr_frame.headertime = header_time;
        curr_frame.frame_id = frame_id;
        curr_frame.setPoints(points_vec);

        //下采样会导致问题
//        curr_frame.grid_sample_mid_in_pcl(DOWNSAMPLE_VOXEL_SIZE);
        curr_frame.Adaptive_sample_mid_in_vec();

        //if seg the cloud scan2scan: use the last full cloud
//        auto scan2scan_start = std::chrono::steady_clock::now();
//        Sophus::SE3d pre_end = curr_frame.pose.end_pose;
//        SCAN2SCAN(curr_frame.points, last_points,curr_frame.pose);
//        Sophus::SE3d aft_end = curr_frame.pose.end_pose;
//        Sophus::SE3d correct = aft_end.inverse()*pre_end;
//        auto scan2scan_end = std::chrono::steady_clock::now();
//        ROS_INFO("SCAN2SCAN SPEND:%f",std::chrono::duration<double,std::milli>(scan2scan_end - scan2scan_start).count());
//        ROS_INFO("CORRECT: %f, %f,%f,%f,%f,%f",correct.angleX(),correct.angleY(),correct.angleZ(),correct.translation().x(),
//                 correct.translation().y(),correct.translation().z());
    }

    bool SearchNeighbor_(const VoxelmapCommon &local_map, const Eigen::Vector3d & PointW,
                         double searchThreshold, std::vector<Eigen::Vector3d> & neighbor_block, int max_neighbor_num=MAX_NEIGHBOR_NUM){
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

                        for(auto  point : voxel_block){
                            Eigen::Vector3d neighbor=point.point_world;

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

                Eigen::Vector3d temp;
                temp[0]=std::get<1>(neighborsQueue.top()).x();
                temp[1]=std::get<1>(neighborsQueue.top()).y();
                temp[2]=std::get<1>(neighborsQueue.top()).z();
                neighbor_block.emplace_back(temp);
                neighborsQueue.pop();
            }
//                 neighbor.addPoint(std::get<1>(neighborsQueue.top()));

            return true;
        }

        return false;

    }

    void computeDistribution(std::vector<Eigen::Vector3d >& neighbor, double & a2D, Eigen::Vector3d & normal){
//        bool isValid = true;
//        if(neighbor.points.size()<5){
//            isValid= false;
//            return ;
//        }
        Eigen::Vector3d braycenter = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();


        for(auto &point:neighbor){
            braycenter += point;
            //cov += (point_ref*point_ref.transpose());
        }

        braycenter /= (double) neighbor.size();

        //faster
        for(auto &point : neighbor){

            for(int k=0;k<3;k++){
                for(int l=k;l<3;l++){
                    cov(k,l)  += (point(k)-braycenter(k)) *(point(l)-braycenter(l));
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

    void Solve(frame_info & curr_frame, int & motion_evaluate){
//            ---------------------------------- solve--------------------------------------------------
        int iter_count = 0;
        double solve_cost=0;
//            vector<double> loss={0.2,0.2,0.2,0.2,0.2};
        double find_neighborcost=0;
        double compouteDisatribute=0;

        for (; iter_count < iter_nums; iter_count++) {

            // ceres opt
            ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.2);
            //ceres::LossFunction * loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem problem;

            Eigen::Quaterniond  begin_quat = curr_frame.beginQuat();
            Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans();
            Eigen::Quaterniond end_quat = curr_frame.endQuat();
            Eigen::Vector3d end_trans = curr_frame.getEndTrans();


            problem.AddParameterBlock(begin_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(end_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold);

            problem.AddParameterBlock(begin_trans.data(), 3);

            problem.AddParameterBlock(end_trans.data(), 3);


            int opt_num=0;
            auto findNeighbor_start = std::chrono::steady_clock::now();

            cloud_valid->clear();
//#pragma omp parallel for num_threads(8)
            for (auto  & p : curr_frame.points) {

                std::vector<Eigen::Vector3d> neighbor;

                double searchDis = 0.8;

                Eigen::Vector4d pabcd;

                p.point_world = curr_frame.pose.linearInplote(p.alpha) * p.point;

                auto time1 = std::chrono::steady_clock::now();
                //bool IsValid=local_map.NeighborSearch(curr_frame.cloud_world->points[i],raw_point,searchDis,neighbor,pabcd);
                bool IsValid= SearchNeighbor_(voxel_map, p.point_world,searchDis,neighbor);
                auto time2 = std::chrono::steady_clock::now();
                find_neighborcost += std::chrono::duration<double, std::milli>(time2 - time1).count();



                if(IsValid){


                    auto time3 = std::chrono::steady_clock::now();
//                        neighbor.computeDescription(A2D | NORMAL);
                    double a2d;
                    Eigen::Vector3d normal;
                    computeDistribution(neighbor,a2d,normal);
                    if(normal.dot(begin_trans-p.point)<0){
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
                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor2, 1, 4, 4, 3, 3>(
                            new CTFunctor2(p.alpha, p.point, neighbor.back(), weight,normal));

                    problem.AddResidualBlock(cost_function,
                                             loss_function,
                                             begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());

                    PointXYZIRT point;
                    point.x = p.point[0];
                    point.y = p.point[1];
                    point.z = p.point[2];
                    cloud_valid->emplace_back(point);
                }
            }
            publishCloud(cloud_valid_pub,cloud_valid,curr_frame.headertime,"odometry");//1ms
//
//            publishCloud(cloud_downsample_pub,curr_frame.cloud_ori_downsample,ros::Time(headertime),"odometry");
            auto findNeighbor_end = std::chrono::steady_clock::now();

           // find_neighborcost += std::chrono::duration<double, std::milli >(findNeighbor_end-findNeighbor_start).count();


            //add other constraints
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistency,6,4,3>(
                                             new LocationConsistency(last_pose.endQuat(), last_pose.endTrans(), std::sqrt(opt_num * 0.01))),
                                     nullptr,
                                     begin_quat.coeffs().data(),begin_trans.data()

            );

            Eigen::Quaterniond pre_quat_delta = last_pose.beginQuat().conjugate() * last_pose.endQuat();
            pre_quat_delta.normalize();

            Eigen::Vector3d pre_trans_delta = last_pose.endTrans() - last_pose.beginTrans();//in world axis

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

            solve_cost += std::chrono::duration<double, std::milli >(solve_end-solve_start).count();


            begin_quat.normalize();
            end_quat.normalize();
            curr_frame.setMotion(begin_quat, end_quat, begin_trans, end_trans);
        }


        if (Debug_print) {
                    ROS_INFO("neighbor COST: %f ms",
                             find_neighborcost);
                    ROS_INFO("compute COST: %f ms",
                             compouteDisatribute);
                    ROS_INFO("solve COST: %f ms",
                             solve_cost);
//                    ROS_INFO("problem make COST: %f ms",
//                             std::chrono::duration<double, std::milli>(findNeighbor_end - findNeighbor_start).count());
//                    ROS_INFO("OPT_NUM:%d",opt_num);


        }

        //last_pose = curr_frame.pose;
        if(motion_evaluate ==2){
            return;
        }
        motion_evaluate = curr_frame.pose.compareDiff(key_poses.back(),10,0.03);
    }

    void ProcessSeg( std::vector<PointType> & points_vec, double timeStart, double timeEnd,
                     double  timeSegBegin, double timeSegEnd, double headertime,
                     int frame_id, int & motion_evaluate,
                     std::vector<PointType> & points_vec_process){

        auto procee_begin = std::chrono::steady_clock::now();

        auto seg_begin = std::chrono::steady_clock::now();
        std::vector<PointType> SegPoints;
        getSegCloud(points_vec,timeStart, timeEnd,
                    timeSegBegin, timeSegEnd,SegPoints);
        auto seg_end =std::chrono::steady_clock::now();

        Sophus::SE3d begin_pose = globalTraj.getLastPose();
        Sophus::SE3d end_pose = globalTraj.predict(timeSegEnd);
        frame_info curr_frame;

        auto init_begin= std::chrono::steady_clock::now();
        Initframe(curr_frame, SegPoints, begin_pose,end_pose, timeSegBegin, timeSegEnd, headertime, frame_id);
        auto init_end = std::chrono::steady_clock::now();

        auto solve_begin = std::chrono::steady_clock::now();
        Solve(curr_frame, motion_evaluate);
        auto solve_end = std::chrono::steady_clock::now();

        auto update_begin = std::chrono::steady_clock::now();
        curr_frame.getWorldPoints(points_vec_process);
        auto update_end = std::chrono::steady_clock::now();

        globalTraj.addPose(curr_frame.pose.begin_pose,curr_frame.timeStart);
        globalTraj.addPose(curr_frame.pose.end_pose,curr_frame.timeEnd);
        last_pose = curr_frame.pose;

        auto process_end = std::chrono::steady_clock::now();


        if(Debug_print){
            ROS_INFO("PROCESS SEG COST:------");
            double  process_cost = std::chrono::duration<double, std::milli > (process_end - process_end).count();
            double  segcost = std::chrono::duration<double, std::milli > (seg_end - seg_begin).count();
            double  initcost = std::chrono::duration<double, std::milli > (init_end - init_begin).count();
            double  solvecost = std::chrono::duration<double, std::milli > (solve_end - solve_begin).count();
            double  updatecost = std::chrono::duration<double, std::milli > (update_end - update_begin).count();
            ROS_INFO("peocess cost all : %f ms \n seg_cost: %f ms \n init_cost: %f ms \n  solve_cost: %f ms \n update_cost: %f ms",
                     process_cost, segcost, initcost, solvecost, updatecost);

        }

    }

    void updateworldPointFromGlobalTrajectory(std::vector<PointType> & points_vec){
        for(auto &point_type : points_vec){
            SE3 point_pose = globalTraj.getPose(point_type.timestamp);
            point_type.point_world = point_pose * point_type.point;
            point_type.point_deskew = globalTraj.getLastPose().inverse() * point_type.point_world;
        }
    }



    void AddCloudToMap(int motion_evaluate_, std::vector<PointType> &points_vec_process, double & header_stamp){
        auto map_start = std::chrono::steady_clock::now();
//            curr_frame.updateWorldCloud();
        //updateworldPointFromGlobalTrajectory(points_vec);
        if( 1||motion_evaluate_==2 ){
//                curr_frame.updateFromDownSample();
//                curr_frame.updateWorldCloud();

            last_points.clear();
            last_points.resize(points_vec_process.size());
            copy(points_vec_process.begin(),points_vec_process.end(),last_points.begin());
//            last_points.clear();
//            points_vec_process.swap(last_points);
            //ROS_INFO("last_points.size : %d",last_points.size());
            voxel_map.updateVoxelMap(points_vec_process);

            key_poses.push_back(last_pose);
            globalMap->clear();
            //getVoxelMap(voxel_map, globalMap);
            voxel_map.getGlobalMap(globalMap);
            //ROS_INFO("GLOBAL_MAP SIZE : %d,",globalMap->size());
            //pub map
            publishCloud(map_pub,globalMap,header_stamp,"map");

        }
        auto map_end = std::chrono::steady_clock::now();
        double map_cost = std::chrono::duration<double,std::milli>(map_end-map_start).count();
        ROS_INFO("MAP UPDATE: %f ms", map_cost);
    }



    void Process()
    {
        ros::Rate r(100);//100Hz

        while (ros::ok())
        {

            if (rosCloudQue.empty())
            {
                continue;
                // return;
            }

            auto all_start = std::chrono::steady_clock::now();

            frame_count++;
            ROS_INFO("------------------------------frame_id:%d----------------------------",frame_count);

            // get pcl cloud

            mtx_get.lock();
            sensor_msgs::PointCloud2 cloud_ori_ros = std::move(rosCloudQue.front());
            rosCloudQue.pop_front();
            mtx_get.unlock();
            //pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori(new pcl::PointCloud<PointXYZIRT>());
            cloud_ori->clear();
            std::vector<PointType> points_vec;
            std::vector<PointType> points_vec_process;


            // get timestamp
            double headertime = cloud_ori_ros.header.stamp.toSec();
            double timeStart = 0.0;
            double timeEnd = 0.0;

//            pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);
//            preprocess(cloud_ori, points_vec,headertime,timeStart, timeEnd);

            CLOUD_CONVERT cc(cloud_ori_ros,(LidarType)lidar_type, points_vec,headertime,timeStart,timeEnd);


            if (first_flag )
            {
                // do some initial work
                //ROS_INFO("1");
                frame_info curr_frame;
                curr_frame.setFrameTime(headertime,timeStart, timeEnd,frame_count);
                curr_frame.pose.initialMotion();

                for(auto &point : points_vec){
                    point.point_world = point.point;
                    point.point_deskew = point.point;
                }

                //build map
                ROS_INFO("BUILD_MAP, Point_size:%d",points_vec.size());

                //build map
                voxel_map.buildVoxelMap(points_vec);

                last_pose=curr_frame.pose;
                key_poses.push_back(curr_frame.pose);

                trajectory_startTime = timeStart;
                globalTraj.addPose(KNOT::Indentity(timeStart, true));
                globalTraj.addPose(KNOT::Indentity(timeEnd));

                //ROS_INFO("2");
                first_flag = false;

                *globalMap += *cloud_ori;

                publish(points_vec, headertime);
                std::vector<PointType>().swap(points_vec);
                //publishCloud(map_pub,globalMap,headertime,"map");
                //ROS_INFO("3");
                continue;
                //return;
            }

            int motion_evaluate =0;

            double timeMid = (timeStart  + timeEnd)/(2.0);
            ROS_INFO("frame time: %f, %f, %f", timeStart, timeMid, timeEnd);
            // init frame_info

//            ProcessSeg(points_vec,timeStart, timeEnd, timeStart, timeEnd, headertime, frame_count, motion_evaluate, points_vec_process);

            ProcessSeg(points_vec,timeStart, timeEnd, timeStart, timeMid, headertime,frame_count, motion_evaluate, points_vec_process);
            ProcessSeg(points_vec,timeStart, timeEnd, timeMid, timeEnd, headertime, frame_count, motion_evaluate, points_vec_process);

            AddCloudToMap(motion_evaluate, points_vec_process, headertime);

            //publish ori_cloud, world_cloud
            publish(points_vec_process,headertime);

//            double timeMid_1 = (timeStart * 2.0 + timeEnd)/(3.0);
//            double timeMid_2 = (timeStart + 2.0 * timeEnd)/(3.0);
//            ROS_INFO("frame time: %f, %f, %f", timeStart, timeMid_1, timeEnd);
//            // init frame_info
//            ProcessSeg(cloud_ori, timeStart, timeMid_1, headertime);
//            ProcessSeg(cloud_ori, timeMid_1, timeMid_2, headertime);
//            ProcessSeg(cloud_ori, timeMid_2, timeEnd, headertime);

            std::vector<PointType>().swap(points_vec);
            std::vector<PointType>().swap(points_vec_process);


            auto all_end = std::chrono::steady_clock::now();
            double all_cost = std::chrono::duration<double, std::milli>(all_end-all_start).count();

            if(Debug_print){
                ROS_INFO("ALL COST: %f ms.", all_cost);
                if(all_cost>cost_threshold){
                    ROS_WARN("COST TOO MUCH!");
                }
            }

            //release


            r.sleep();
        }
    }


    void publishCloud(ros::Publisher &pub, pcl::PointCloud<PointXYZIRT>::ConstPtr pcl_cloud, double &header_stamp ,string frame){
        ros::Time ros_headertime = ros::Time(header_stamp);
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_cloud, ros_cloud);
        ros_cloud.header.stamp = ros_headertime;
        ros_cloud.header.frame_id = frame;
        pub.publish(ros_cloud);
        return ;
    }

    void transVec2Pcl(std::vector<PointType> &points_vec){
        cloud_world->clear();
        cloud_deskew->clear();
        cloud_ori->clear();
        for(auto & point_type : points_vec){
            Eigen::Vector3d world_point = point_type.point_world;
            Eigen::Vector3d deskew_point = point_type.point_deskew;
            Eigen::Vector3d point = point_type.point;
            PointXYZIRT p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            p.intensity = point_type.intensity;
            cloud_ori->points.emplace_back(p);
            p.x = world_point[0];
            p.y = world_point[1];
            p.z = world_point[2];
            cloud_world->points.emplace_back(p);
            p.x = deskew_point[0];
            p.y = deskew_point[1];
            p.z = deskew_point[2];
            cloud_deskew->points.emplace_back(p);
        }
    }

    void publish(std::vector<PointType> & points_vec, double headertime) {
        transVec2Pcl(points_vec);
        nav_msgs::Odometry odometry_pose;
        odometry_pose.header.stamp = ros::Time(headertime);
        odometry_pose.header.frame_id = odometry_frame;
        odometry_pose.pose.pose.orientation.w = last_pose.endQuat().w();
        odometry_pose.pose.pose.orientation.x = last_pose.endQuat().x();
        odometry_pose.pose.pose.orientation.y = last_pose.endQuat().y();
        odometry_pose.pose.pose.orientation.z = last_pose.endQuat().z();

        odometry_pose.pose.pose.position.x = last_pose.endTrans().x();
        odometry_pose.pose.pose.position.y = last_pose.endTrans().y();
        odometry_pose.pose.pose.position.z = last_pose.endTrans().z();


        geometry_msgs::PoseStamped geo_odometry_pose;
        geo_odometry_pose.header = odometry_pose.header;
        geo_odometry_pose.pose = odometry_pose.pose.pose;

        globalPath.header.stamp = ros::Time(headertime);
        globalPath.header.frame_id = "map";
        globalPath.poses.emplace_back(geo_odometry_pose);

        sensor_msgs::PointCloud2 deskew_cloud;
        pcl::toROSMsg(*cloud_deskew,deskew_cloud);
        deskew_cloud.header.stamp = ros::Time(headertime);
        deskew_cloud.header.frame_id= "odometry";

        sensor_msgs::PointCloud2 world_cloud;
        pcl::toROSMsg(*cloud_world,world_cloud);
        world_cloud.header.stamp = ros::Time(headertime);
        world_cloud.header.frame_id="map";


        sensor_msgs::PointCloud2 ori_cloud;
        pcl::toROSMsg(*cloud_ori,ori_cloud);
        ori_cloud.header.stamp = ros::Time(headertime);
        ori_cloud.header.frame_id= "odometry";
        traj_pub.publish(globalPath);
        odo_pub.publish(odometry_pose);
        cloud_ori_pub.publish(ori_cloud);
        cloud_pub.publish(deskew_cloud);
        cloud_world_pub.publish(world_cloud);







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
    std::thread process(&Lidar_odo::Process, &lp);
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