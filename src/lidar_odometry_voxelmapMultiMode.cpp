#include "AdaCt/utility.h"
//#include "AdaCt/frame_info.hpp"

#include "AdaCt/frame_infoMultiMode.hpp"

//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmapMultiMode.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;


const bool timelist(PointXYZIRT &x, PointXYZIRT &y){return x.timestamp < y.timestamp;}

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


    // cloud
    std::deque<sensor_msgs::PointCloud2> rosCloudQue;
    sensor_msgs::PointCloud2 cloud_ori_ros;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;

    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

    double timeStart, timeEnd, headertime, timeMid;
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

    std::vector<OptPoseMultiMode> poses;
    std::vector<double> timeVec;
    int iter_nums; // 5
    int node_num;

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


        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());

        first_flag = true;
        frame_count =-1;

        iter_nums =5;

        rosCloudQue.clear();

        node_num=3;
        curr_frame.setNode(node_num);

//        pre_pose.initialMotion();
//        curr_pose.initialMotion();

        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());


    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
    }

    void Initframe()
    {

        curr_frame.setFrameTime(headertime,timeVec, frame_count);
        curr_frame.setOricloud(cloud_ori);
        if(frame_count<=1){
            curr_frame.normalize();
            //poses.push_back(curr_frame.pose);
        }else{

//            curr_frame.pose.begin_pose = poses[frame_count].begin_pose *(poses[frame_count-1].begin_pose.inverse() * poses[frame_count].begin_pose);
//            curr_frame.pose.end_pose = poses[frame_count].end_pose * (poses[frame_count-1].end_pose.inverse() * poses[frame_count].end_pose);
//            curr_frame.pose.begin_pose = poses.back().end_pose;
//            curr_frame.pose.end_pose = curr_frame.pose.begin_pose *(poses.back().begin_pose.inverse()*poses.back().end_pose);
              curr_frame.Propagate(poses.back());
        }

//        curr_frame.pose.begin_pose = poses.back().end_pose;
//        //curr_frame.pose.end_pose = poses[frame_count-1].end_pose * poses[frame_count-2].end_pose.inverse() * poses[frame_count-1].end_pose;
//        curr_frame.pose.end_pose = curr_frame.pose.begin_pose *(poses.back().begin_pose.inverse()*poses.back().end_pose);
//        curr_frame.pose=poses.back();


        //curr_frame.downSampleOriCloud();
        //下采样会导致问题
        curr_frame.grid_sample_mid();
        //curr_frame.update();
        curr_frame.updateFromDownSample();
    }

    void preprocess()
    {
        ros::Rate r(100);
        while (ros::ok())
        {

            if (rosCloudQue.size() == 0)
            {
                continue;
            }

            frame_count++;
            ROS_INFO("frame_id:%d",frame_count);
            // get pcl cloud
            cloud_ori_ros = std::move(rosCloudQue.front());
            rosCloudQue.pop_front();

            pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);

            auto initframe_start = std::chrono::steady_clock::now();
            //std::vector<int> indices;
            //pcl::removeNaNFromPointCloud(*cloud_ori, *cloud_ori, indices);
            std::sort(cloud_ori->points.begin(),cloud_ori->points.end(), timelist);


            // get timestamp
            headertime = cloud_ori_ros.header.stamp.toSec();

            timeStart = cloud_ori->points[0].timestamp;
            timeEnd = cloud_ori->points.back().timestamp;
            timeMid = (timeStart+timeEnd)/2;

            timeVec={timeStart,timeMid,timeEnd};
            //kd_map.delete_margincube(curr_frame.getEndTrans());

            if (first_flag )
            {
                // do some initial work

                curr_frame.setFrameTime(headertime,timeVec, frame_count);
                curr_frame.setOricloud(cloud_ori);
                curr_frame.normalize();

                local_map.InsertPointCloud(curr_frame.cloud_ori,curr_frame.poses);

                poses.push_back(curr_frame.poses);

                first_flag = false;

                *globalMap += *cloud_ori;
                ROS_INFO("init");
                publish();
                continue;
            }

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
                ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
                ceres::Problem problem;

                Eigen::Quaterniond  begin_quat = curr_frame.beginQuat();
                Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans();
                Eigen::Quaterniond end_quat = curr_frame.endQaut();
                Eigen::Vector3d end_trans = curr_frame.getEndTrans();
                Eigen::Quaterniond  mid_quat = curr_frame.poses[1].unit_quaternion();
                Eigen::Vector3d mid_trans = curr_frame.poses[1].translation();


                double *begin_quat_param = curr_frame.beginQuat().coeffs().data();
                double *end_quat_param = curr_frame.endQaut().coeffs().data();
                double *begin_trans_param = curr_frame.getBeginTrans().data();
                double *end_trans_param = curr_frame.getEndTrans().data();


//                problem.AddParameterBlock(begin_quat.coeffs().data(),4);
//                problem.SetManifold(begin_quat.coeffs().data(), new ceres::EigenQuaternionManifold);
//
//                problem.AddParameterBlock(end_quat.coeffs().data(), 4);
//                problem.SetManifold(end_quat.coeffs().data(),new ceres::EigenQuaternionManifold);


                problem.AddParameterBlock(begin_quat.coeffs().data(),4, new ceres::EigenQuaternionParameterization());
                problem.AddParameterBlock(end_quat.coeffs().data(),4, new ceres::EigenQuaternionParameterization());
                problem.AddParameterBlock(mid_quat.coeffs().data(),4,new ceres::EigenQuaternionParameterization());

                problem.AddParameterBlock(begin_trans.data(), 3);
                problem.AddParameterBlock(mid_trans.data(),3);

                problem.AddParameterBlock(end_trans.data(), 3);

                // add parameterblock
                // auto & begin_pose=curr_frame.pose.begin_pose;
                // auto & end_pose=curr_frame.pose.end_pose;

                // problem.AddParameterBlock(begin_pose.data(),Sophus::SE3d::num_parameters, new LocalParameterizationSE3);
                // problem.AddParameterBlock(end_pose.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3);


                int cloud_size = curr_frame.cloud_ori_downsample->size();

                auto findNeighbor_start = std::chrono::steady_clock::now();
                int opt_num=0;
                cloud_valid->clear();

//#pragma omp parallel for num_threads(8)
                for (int i = 0; i < cloud_size; i++) {


                    //Neighbors_queue neighborsQueue;
                    VoxelBlock<PointXYZIRT> neighbor;

                    double searchDis = 0.8;

                    Eigen::Vector4d pabcd;

                    double alpha = (curr_frame.cloud_ori_downsample->points[i].timestamp - curr_frame.timeStart)/( curr_frame.timeEnd- curr_frame.timeStart);
                    Eigen::Vector3d raw_point(curr_frame.cloud_ori_downsample->points[i].x,curr_frame.cloud_ori_downsample->points[i].y,curr_frame.cloud_ori_downsample->points[i].z);


                    Eigen::Vector3d sensor_local(curr_frame.cloud_world->points[i].x,curr_frame.cloud_world->points[i].y,
                                                    curr_frame.cloud_world->points[i].z);

//                    Eigen::Quaterniond inter_qual = begin_quat.normalized().slerp(alpha, end_quat.normalized());
//
//                    Eigen::Vector3d world_points = inter_qual * raw_point + sensor_local;

                    if(local_map.NeighborSearch(curr_frame.cloud_world->points[i],sensor_local,searchDis,neighbor,pabcd)){
                        opt_num++;

                        neighbor.computeDescription(A2D | NORMAL);
//                        double weight=0.9*std::pow(neighbor.description.a2D,2)+
//                                0.1*std::exp(-(getPointXYZ(neighbor.points[0])- getPointXYZ(curr_frame.cloud_world->points[i])).norm()/(0.3*5));
                        double weight =neighbor.description.a2D;
//                        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor, 1, 4, 4, 3, 3>(
//                                new CTFunctor(alpha, raw_point, pabcd, weight));
//                        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor2, 1, 4, 4, 3, 3>(
//                                new CTFunctor2(alpha, raw_point, getPointXYZ(neighbor.points[0]), weight,neighbor.description.normal));
                        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor3, 1, 4, 4,4,3, 3, 3>(
                                new CTFunctor3(alpha, raw_point, getPointXYZ(neighbor.points[0]), weight,neighbor.description.normal));

                        problem.AddResidualBlock(cost_function,
                                                 loss_function,
                                                 begin_quat.coeffs().data(),mid_quat.coeffs().data(),end_quat.coeffs().data(),
                                                 begin_trans.data(),mid_trans.data(), end_trans.data());
                        cloud_valid->push_back(curr_frame.cloud_ori_downsample->points[i]);
                    }
                }


                publishCloud(cloud_valid_pub,cloud_valid,ros::Time(headertime),"odometry");
                auto findNeighbor_end = std::chrono::steady_clock::now();

                if (debug_print) {
                    ROS_INFO("cloud find neighbor COST: %f ms",
                             std::chrono::duration<double, std::milli>(findNeighbor_end - findNeighbor_start).count());
                }
                //add other constraints
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistency,3,3>(
                        new LocationConsistency(poses.back().endTrans(),std::sqrt(opt_num*0.001))),
                                         nullptr,
                                         begin_trans.data()

                        );
                //v consist
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<MultiModeConstantVelocity, 6, 3, 3, 3>(
                                                 new MultiModeConstantVelocity(
                                                         poses.back().endTrans() - poses.back().poses[1].translation(),
                                                         std::sqrt(opt_num * 0.001))),
                                         nullptr,
                                         begin_trans.data(), mid_trans.data(), end_trans.data()
                );

                //w?




                // solve
                ceres::Solver::Options option;
                option.linear_solver_type = ceres::DENSE_QR;
                option.max_num_iterations =4;
                option.num_threads = 16;
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
                mid_quat.normalize();
                end_quat.normalize();
                std::vector<Eigen::Quaterniond> quat_vec={begin_quat,mid_quat,end_quat};
                std::vector<Eigen::Vector3d > trans_vec={begin_trans,mid_trans,end_trans};
                curr_frame.setMotion(quat_vec,trans_vec);
                curr_frame.updateFromDownSample();
                //curr_frame.normalize();

                //curr_frame.update();


//                if(curr_frame.pose.compareDiff(poses.back())) {
//                    poses.push_back(curr_frame.pose);
//                    break;
//                }




            }
//            EulerAngles end_euler = ToEulerAngles(pre_pose.endQuat());
//            ROS_INFO("ruler: %f, %f,%f", end_euler.roll, end_euler.pitch, end_euler.yaw);

            // update local_map
            auto map_start = std::chrono::steady_clock::now();
            local_map.RemoveFarFromLocation(curr_frame.getEndTrans(),300);
            local_map.InsertPointCloud(curr_frame.cloud_world,curr_frame.poses);
            poses.push_back(curr_frame.poses);
            auto map_end = std::chrono::steady_clock::now();
            ROS_INFO("MAP UPDATE COST: %f ms",std::chrono::duration<double,std::milli>(map_end-map_start).count());
            //updateLocalMap(curr_frame,neighbor);
//            ROS_INFO("MAP SIZE: %d", local_map.size());
            *globalMap += *curr_frame.cloud_world;

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
    ROS_INFO("START LIDAR PREPROCESS!");

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