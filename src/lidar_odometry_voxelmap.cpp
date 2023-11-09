#include "AdaCt/utility.h"
#include "AdaCt/frame_info.hpp"

//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "AdaCt/voxelmap.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;


const bool timelist(PointXYZIRT &x, PointXYZIRT &y){return x.timestamp < y.timestamp;}

//



class Lidar_preprocess : public configParam
{
public:
    ros::Subscriber lidar_sub;

    ros::Publisher map_pub;
    ros::Publisher odo_pub;
    ros::Publisher traj_pub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_ori_pub;
    ros::Publisher cloud_valid_pub;


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

    // debug info
    int beforeRemoveNan, afterRomoveNan;

    // map
    Voxelmap<PointXYZIRT> local_map;
    // opt pose
    OptPose pre_pose, curr_pose;
    int iter_nums; // 5

    Lidar_preprocess()
    {
        //ROS_INFO("1");
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_preprocess::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/deskew_pointcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);


        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());

        first_flag = true;
        frame_count = 0;

        iter_nums =5;

        rosCloudQue.clear();

        pre_pose.initialMotion();
        curr_pose.initialMotion();

        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());


    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
    }

    void Initframe()
    {

        curr_frame.setFrameTime(headertime, timeStart, timeEnd, frame_count);
        curr_frame.setOricloud(cloud_ori);
        curr_frame.setMotion(pre_pose);

        //curr_frame.downSampleOriCloud();
        //下采样会导致问题
        curr_frame.update();
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
            ROS_INFO("frame_id:%d",frame_count);
            frame_count++;

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

            //kd_map.delete_margincube(curr_frame.getEndTrans());

            if (first_flag )
            {
                // do some initial work

                curr_frame.setFrameTime(headertime, timeStart, timeEnd, frame_count);
                curr_frame.setOricloud(cloud_ori);

                local_map.InsertPointCloud(curr_frame.cloud_ori);

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
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem problem;

                Eigen::Quaterniond  begin_quat = curr_frame.beginQuat().normalized();
                Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans().normalized();
                Eigen::Quaterniond end_quat = curr_frame.endQaut();
                Eigen::Vector3d end_trans = curr_frame.getEndTrans();


                problem.AddParameterBlock(begin_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold());
                problem.AddParameterBlock(end_quat.coeffs().data(), 4, new ceres::EigenQuaternionManifold());
                problem.AddParameterBlock(begin_trans.data(), 3);
                problem.AddParameterBlock(end_trans.data(), 3);

                // add parameterblock
                // auto & begin_pose=curr_frame.pose.begin_pose;
                // auto & end_pose=curr_frame.pose.end_pose;

                // problem.AddParameterBlock(begin_pose.data(),Sophus::SE3d::num_parameters, new LocalParameterizationSE3);
                // problem.AddParameterBlock(end_pose.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3);


                int cloud_size = curr_frame.cloud_ori->size();

                auto findNeighbor_start = std::chrono::steady_clock::now();
                int opt_num=0;
                cloud_valid->clear();
                for (int i = 0; i < cloud_size; i++) {


                    Neighbors_queue neighborsQueue;

                    double searchDis = 0.8;

                    Eigen::Vector4d pabcd;
                    if(local_map.NeighborSearch(curr_frame.cloud_world->points[i],searchDis,neighborsQueue,pabcd)){
                        opt_num++;
                        double alpha = (curr_frame.cloud_ori->points[i].timestamp - curr_frame.timeStart)/( curr_frame.timeEnd- curr_frame.timeStart);
                        Eigen::Vector3d raw_point(curr_frame.cloud_ori->points[i].x,curr_frame.cloud_ori->points[i].y,curr_frame.cloud_ori->points[i].z);
                        double weight=1.0;
                        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor, 1, 4, 4, 3, 3>(
                                new CTFunctor(alpha, raw_point, pabcd, weight));

                        problem.AddResidualBlock(cost_function,
                                                 loss_function,
                                                 begin_quat.coeffs().data(), end_quat.coeffs().data(), begin_trans.data(), end_trans.data());
                        cloud_valid->push_back(curr_frame.cloud_ori->points[i]);
                    }

                }

                publishCloud(cloud_valid_pub,cloud_valid,ros::Time(headertime),"odometry");
                auto findNeighbor_end = std::chrono::steady_clock::now();

                if (debug_print) {
                    ROS_INFO("cloud find neighbor COST: %f ms",
                             std::chrono::duration<double, std::milli>(findNeighbor_end - findNeighbor_start).count());
                }

                // solve
                ceres::Solver::Options option;
                option.linear_solver_type = ceres::DENSE_QR;
                option.max_num_iterations = 4;
                option.num_threads = 12;
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


                curr_frame.setMotion(begin_quat.normalized(), end_quat.normalized(), begin_trans, end_trans);
                curr_frame.normalize();

                curr_frame.update();

//                if(curr_frame.pose.compareDiff(pre_pose)) {
//                    pre_pose = curr_frame.pose;
//                    break;
//                }

                pre_pose = curr_frame.pose;


            }
            EulerAngles end_euler = ToEulerAngles(pre_pose.endQuat());
            ROS_INFO("ruler: %f, %f,%f", end_euler.roll, end_euler.pitch, end_euler.yaw);

            // update local_map
            auto map_start = std::chrono::steady_clock::now();
            local_map.RemoveFarFromLocation(curr_frame.getEndTrans(),300);
            local_map.InsertPointCloud(curr_frame.cloud_world);
            auto map_end = std::chrono::steady_clock::now();
            ROS_INFO("MAP UPDATE COST: %f ms",std::chrono::duration<double,std::milli>(map_end-map_start).count());
            //updateLocalMap(curr_frame,neighbor);
            ROS_INFO("MAP SIZE: %d", local_map.size());
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

        sensor_msgs::PointCloud2 ori_cloud;
        pcl::toROSMsg(*curr_frame.cloud_ori,ori_cloud);
        ori_cloud.header.stamp = ros::Time(headertime);
        ori_cloud.header.frame_id= "odometry";

        cloud_ori_pub.publish(ori_cloud);
        cloud_pub.publish(deskew_cloud);

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
    Lidar_preprocess lp;
//
//    // // 由于process是循环，如果直接运行，不会进入下面的spin，就不会进入点云回调函数，所以要新开一个线程
    std::thread process(&Lidar_preprocess::preprocess, &lp);
//
//    // // spin才会进入回调函数
    ros::spin();
//
//    // // 加入线程
     process.join();

    return 0;
}