#include "AdaCt/utility.h"
#include "AdaCt/frame_info_usecommonPointType.hpp"

//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap_OctoTree.hpp"
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
    sensor_msgs::PointCloud2 cloud_ori_ros;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;

    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

    double timeStart, timeEnd, headertime;
    frame_info curr_frame;

    std::mutex mtx;

    bool first_flag;
    int frame_count;

    // map
    tsl::robin_map<Voxel, OctoTree*> voxel_map;
    float max_voxel_size;
    int max_layer;
    vector<int> layer_size;
    int max_points_size;
    float min_eigen_value;

    std::vector<OptPose> key_poses;
    OptPose last_pose;
    std::vector<PointType> last_points;


    int iter_nums; // 5

    //time cost ms
    double init_cost;
    double neighbor_find_average;
    double solve_average;
    double solve_cost;
    double map_cost;
    double all_cost;
    double cost_threshold;
    double cost_robust;

    int motion_evaluate;
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

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());
        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());

        rosCloudQue.clear();

        //time cost
        init_cost = 0;
        neighbor_find_average = 0;
        solve_average = 0;
        solve_cost = 0;
        map_cost = 0;
        all_cost = 0;
        cost_threshold = 100.0;
        cost_robust = 30.0;


        // params init
        first_flag = true;
        frame_count =-1;
        iter_nums =MAX_ITER_COUNT;

        motion_evaluate=0;
        last_pose.initialMotion();
        //mapping params
        max_voxel_size = 0.8;
        max_layer = 1;//0,1,2
        layer_size=vector<int>{10,5,5,5,5};
        max_points_size = 1000;
        min_eigen_value = 0.01;


        last_points.clear();

    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
        //preprocess();
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
        for(int iter =0; iter<MAX_ITER_COUNT_SCAN;iter++){
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
    void Initframe()
    {
        curr_frame.Reset();
        curr_frame.setOricloud(cloud_ori);
        curr_frame.setFrameTime(headertime,timeStart,timeEnd,frame_count);
        if(frame_count<1){
            curr_frame.pose.initialMotion();
            //key_poses.push_back(curr_frame.pose);
        }else{
            curr_frame.pose.begin_pose = last_pose.end_pose;
            curr_frame.pose.end_pose = curr_frame.pose.begin_pose *(last_pose.begin_pose.inverse() * last_pose.end_pose);
            //curr_frame.pose.end_pose = key_poses.back().end_pose *(key_poses[key_poses.size()-2].end_pose.inverse() * key_poses.back().end_pose);
        }


        //curr_frame.getTimeStamp(headertime, timeStart, timeEnd);

        //下采样会导致问题
//        curr_frame.grid_sample_mid_in_pcl(DOWNSAMPLE_VOXEL_SIZE);
        curr_frame.Adaptive_sample_mid_in_pcl();

        auto scan2scan_start = std::chrono::steady_clock::now();
        Sophus::SE3d pre_end = curr_frame.pose.end_pose;
        SCAN2SCAN(curr_frame.points, last_points,curr_frame.pose);
        Sophus::SE3d aft_end = curr_frame.pose.end_pose;
        Sophus::SE3d correct = aft_end.inverse()*pre_end;
        auto scan2scan_end = std::chrono::steady_clock::now();
        ROS_INFO("SCAN2SCAN SPEND:%f",std::chrono::duration<double,std::milli>(scan2scan_end - scan2scan_start).count());
        ROS_INFO("CORRECT: %f, %f,%f,%f,%f,%f",correct.angleX(),correct.angleY(),correct.angleZ(),correct.translation().x(),
                 correct.translation().y(),correct.translation().z());
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
            ROS_INFO("------------------------------frame_id:%d----------------------------",frame_count);

            // get pcl cloud
            motion_evaluate =0;
            cloud_ori_ros = std::move(rosCloudQue.front());
            rosCloudQue.pop_front();


            cloud_ori->clear();
            timeStart=0.0;
            timeEnd =0.0 ;
            pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);
//           ---------------------------------------------------init------------------------------------
            auto initframe_start = std::chrono::steady_clock::now();

            // get timestamp
            headertime = cloud_ori_ros.header.stamp.toSec();
            sort(cloud_ori->points.begin(),cloud_ori->points.end(), timelist);
            timeStart = cloud_ori->points[0].timestamp;
            timeEnd = cloud_ori->points.back().timestamp;

            if (first_flag )
            {
                // do some initial work
                //ROS_INFO("1");
                curr_frame.setFrameTime(headertime, timeStart, timeEnd, frame_count);
                curr_frame.setOricloud(cloud_ori);
                curr_frame.pose.initialMotion();

                std::vector<PointType> pv_list;
                for(auto &point : cloud_ori->points){
                    PointType p;
                    //delete the nan point
                    if(!std::isfinite(point.x)|| !std::isfinite(point.y) || !std::isfinite(point.z)){
                        continue;
                    }
                    p.point<< point.x, point.y, point.z;
                    p.point_world<<point.x,point.y,point.z;
                    pv_list.push_back(p);
                }
                //build map
                ROS_INFO("BUILD_MAP, Point_size:%d",pv_list.size());
                buildVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                              max_points_size, max_points_size, min_eigen_value,
                              voxel_map);

                last_pose=curr_frame.pose;
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
            Initframe();
            //cloud_ori->clear();
            auto initframe_end = std::chrono::steady_clock::now();
            init_cost = std::chrono::duration<double, std::milli>(initframe_end-initframe_start).count();
            if(debug_print) {
                ROS_INFO("After DownSample: %d",curr_frame.cloud_ori_downsample->size());
            }
            ROS_INFO("INIT COST: %f ms", init_cost);

//            ---------------------------------- solve--------------------------------------------------
            int iter_count = 0;
            auto iter_start = std::chrono::steady_clock::now();
            for (; iter_count < iter_nums; iter_count++) {

                // ceres opt
                ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
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
                double find_neighborcost=0;

                cloud_valid->clear();
                auto time1 = std::chrono::steady_clock::now();
                //bool IsValid=local_map.NeighborSearch(curr_frame.cloud_world->points[i],raw_point,searchDis,neighbor,pabcd);
                vector<ptpl> ptpl_list;
                vector<Eigen::Vector3d > no_match_list;

                for(auto & p: curr_frame.points){
                    p.point_world = curr_frame.pose.linearInplote(p.alpha) * p.point;
                }

                //ptpl->point is point-body
                BuildResidualListOMP(voxel_map,max_voxel_size,3.0,max_layer, curr_frame.points,ptpl_list,no_match_list);
                auto time2 = std::chrono::steady_clock::now();
                find_neighborcost += std::chrono::duration<double, std::milli>(time2 - time1).count();
                opt_num = ptpl_list.size();//0
//                if(Debug_print){
//                    ROS_INFO("all_point:%d, valid_num: %d",curr_frame.points.size(), opt_num);
//                }

                for(auto  &pl : ptpl_list){

                    double alpha  = pl.point_alpha;
                    double weight = pl.A2D;

//                    if(pl.normal.dot(begin_trans - pl.point)<0){
//                        pl.normal = -1.0 * pl.normal;
//                    }
                    if(pl.normal.dot(pl.center - pl.point_world)<0){
                        pl.normal = -1.0 * pl.normal;
                    }
//                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor4, 1, 4, 4, 3, 3>(
//                            new CTFunctor4(alpha, pl.point, pl.d, weight,pl.normal));
                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor2, 1, 4, 4, 3, 3>(
                            new CTFunctor2(alpha, pl.point, pl.center, weight,pl.normal));

                    problem.AddResidualBlock(cost_function,
                                             loss_function,
                                             begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());

                    PointXYZIRT p;
                    p.x = pl.point[0];
                    p.y = pl.point[1];
                    p.z = pl.point[2];
                    cloud_valid->push_back(p);
                }



                publishCloud(cloud_valid_pub,cloud_valid,ros::Time(headertime),"odometry");//1ms

                publishCloud(cloud_downsample_pub,curr_frame.cloud_ori_downsample,ros::Time(headertime),"odometry");
                auto findNeighbor_end = std::chrono::steady_clock::now();

                neighbor_find_average = std::chrono::duration<double, std::milli >(findNeighbor_end-findNeighbor_start).count();

                if (Debug_print) {
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
                        new LocationConsistency(last_pose.endQuat(), last_pose.endTrans(), std::sqrt(opt_num * 0.01))),
                                         nullptr,
                                         begin_quat.coeffs().data(),begin_trans.data()

                        );


                Eigen::Quaterniond pre_quat_delta = last_pose.beginQuat().conjugate() * last_pose.endQuat();
                pre_quat_delta.normalize();
                Eigen::Vector3d pre_trans_delta = last_pose.endTrans() - last_pose.beginTrans();//in world axis
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocityRotTran,6,4,4,3,3>(
                        new ConstantVelocityRotTran(pre_quat_delta,pre_trans_delta,std::sqrt(opt_num*0.01))),
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

                begin_quat.normalize();
                end_quat.normalize();
                curr_frame.setMotion(begin_quat, end_quat, begin_trans, end_trans);

                motion_evaluate = curr_frame.pose.compareDiff(last_pose);
//                last_pose = curr_frame.pose;

                if(motion_evaluate ==0){
                    break;
                }

            }
            auto iter_end = std::chrono::steady_clock::now();

            solve_cost = std::chrono::duration<double,std::milli>(iter_end - iter_start).count();

            if(Debug_print)
            {
                ROS_INFO("finish with %d iters.", iter_count);
                ROS_INFO("find neighbor cost : %f",neighbor_find_average);
                ROS_INFO("solve : %f", solve_average);
            }
            ROS_INFO("solve cost:%f",solve_cost);
//          --------------------------------------update local_map---------------------------------------
            auto map_start = std::chrono::steady_clock::now();
//            curr_frame.updateWorldCloud();
            motion_evaluate = curr_frame.pose.compareDiff(key_poses.back());
            last_pose = curr_frame.pose;

            //
            if(motion_evaluate==2 ){
//                curr_frame.updateFromDownSample();
//                curr_frame.updateWorldCloud();
                last_points.clear();
                curr_frame.updateFromDownSample();
                last_points = curr_frame.points;
                updateVoxelMap(curr_frame.points, max_voxel_size, max_layer, layer_size,
                               max_points_size, max_points_size, min_eigen_value,
                               voxel_map);

                key_poses.push_back(curr_frame.pose);
                globalMap->clear();
                getVoxelMap(voxel_map, globalMap);

            }
            auto map_end = std::chrono::steady_clock::now();
            map_cost = std::chrono::duration<double,std::milli>(map_end-map_start).count();

            //get map
            //*globalMap += *curr_frame.cloud_world;

            // publish map
            publish();
            // debugPrint();

            auto all_end = std::chrono::steady_clock::now();

            all_cost = std::chrono::duration<double, std::milli>(all_end-all_start).count();



            ROS_INFO("MAP UPDATE COST: %f ms",map_cost);
            ROS_INFO("ALL COST: %f ms",all_cost);


            if(Debug_print){
                if(all_cost>cost_threshold){
                    ROS_WARN("COST TOO MUCH!");
                }
            }


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
        odometry_pose.pose.pose.orientation.w = curr_frame.endQuat().w();
        odometry_pose.pose.pose.orientation.x = curr_frame.endQuat().x();
        odometry_pose.pose.pose.orientation.y = curr_frame.endQuat().y();
        odometry_pose.pose.pose.orientation.z = curr_frame.endQuat().z();

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

        odo_pub.publish(odometry_pose);
        cloud_ori_pub.publish(ori_cloud);
        cloud_pub.publish(deskew_cloud);
        cloud_world_pub.publish(world_cloud);

        traj_pub.publish(globalPath);





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