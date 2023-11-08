#include "AdaCt/utility.h"
//#include "AdaCt/frame_info.hpp"
#include "AdaCt/kdmap.hpp"
//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"


//kdMap have to put before main()
kdMap kd_map;
//KD_TREE<pcl::PointXYZ> ikd_tree;


const bool timelist(PointXYZIRT &x, PointXYZIRT &y){return x.timestamp < y.timestamp;}
bool esti_plane(Eigen::Vector4d &pabcd, std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &neighbor_i, double threshold)
{
    Eigen::Matrix<double, num_neighbor, 3> A;
    Eigen::Matrix<double, num_neighbor, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int i = 0; i < num_neighbor; i++)
    {
        A(i, 0) = neighbor_i[i].x;
        A(i, 1) = neighbor_i[i].y;
        A(i, 2) = neighbor_i[i].z;
    }

    Eigen::Matrix<double, 3, 1> norm_vec = A.colPivHouseholderQr().solve(b);

    double n = norm_vec.norm();

    pabcd(0) = norm_vec(0) / n;
    pabcd(1) = norm_vec(1) / n;
    pabcd(2) = norm_vec(2) / n;
    pabcd(3) = 1.0 / n;

    for (int j = 0; j < num_neighbor; j++)
    {
        if (fabs(pabcd(0) * neighbor_i[j].x + pabcd(1) * neighbor_i[j].y + pabcd(2) * neighbor_i[j].z + pabcd(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}
//



class Lidar_preprocess : public configParam
{
public:
    ros::Subscriber lidar_sub;

    ros::Publisher map_pub;
    ros::Publisher odo_pub;
    ros::Publisher traj_pub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_downsample;


    // cloud
    std::deque<sensor_msgs::PointCloud2> rosCloudQue;
    sensor_msgs::PointCloud2 cloud_ori_ros;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori;

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
    //kdMap kd_map;
     //KD_TREE<pcl::PointXYZ> ikd_tree;
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
        cloud_downsample = nh.advertise<sensor_msgs::PointCloud2>("adact/cloud_downsample",1);

        cloud_ori.reset(new pcl::PointCloud<PointXYZIRT>());

        first_flag = true;
        frame_count = 0;

        iter_nums =3;

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
        curr_frame.grid_sample();
        //下采样会导致问题
//        curr_frame.update();
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
            ROS_INFO("frame_id:%d",frame_count);
            frame_count++;

            // get pcl cloud
            cloud_ori_ros = std::move(rosCloudQue.front());
            rosCloudQue.pop_front();

            pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);

            auto initframe_start = std::chrono::steady_clock::now();
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_ori, *cloud_ori, indices);
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

                kd_map.localMapInit(cloud_ori);
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

            std::vector<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>> neighbor;
            neighbor.resize(curr_frame.cloud_ori_downsample->size());

            for (int iter_count = 0; iter_count < iter_nums; iter_count++) {
                //neighbor.clear();//size=0
                // find correspance

                // ceres opt
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem problem;

                Eigen::Quaterniond  begin_quat = curr_frame.beginQuat();
                Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans();
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

                pcl::PointCloud<PointXYZIRT>::Ptr norm_vec(new pcl::PointCloud<PointXYZIRT>());
                std::vector<bool> HaveValidPlane;
                int cloud_size = curr_frame.cloud_ori_downsample->size();
                int valid_size=0;
                norm_vec->clear();
                norm_vec->resize(cloud_size);
                // int num_neighbor=5;
                auto findNeighbor_start = std::chrono::steady_clock::now();
                for (int i = 0; i < cloud_size; i++) {
                    std::vector<float> pointSearchSqDis(num_neighbor);

                    int pointcloud_size = curr_frame.cloud_ori_downsample->points.size();
                    HaveValidPlane = std::vector<bool>(pointcloud_size, false);

                    kd_map.searchNeighbor(curr_frame.cloud_world->points[i], neighbor[i], pointSearchSqDis);

                    if (neighbor[i].size() < num_neighbor || pointSearchSqDis[num_neighbor - 1] > 5) {

                        continue;
                    }
                    Eigen::Vector4d pabcd;
                    PointXYZIRT point_world = curr_frame.cloud_world->points[i];

                    // use ori or deskew?
                    Eigen::Vector3d point_body(curr_frame.cloud_ori_downsample->points[i].x, curr_frame.cloud_ori_downsample->points[i].y,
                                               curr_frame.cloud_ori_downsample->points[i].z);
                    if (esti_plane(pabcd, neighbor[i], 0.1f)) {
                        double point2plane =
                                pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z +
                                pabcd(3);
                        double s = 1 - 0.9 * fabs(point2plane) / sqrt(point_body.norm());

                        if (s > 0.9) {
                            HaveValidPlane[i] = true;
                            valid_size++;
                            norm_vec->points[i].x = pabcd(0);
                            norm_vec->points[i].y = pabcd(1);
                            norm_vec->points[i].z = pabcd(2);
                            norm_vec->points[i].intensity = point2plane;
                            norm_vec->points[i].timestamp = curr_frame.cloud_ori->points[i].timestamp;

                            double alpha = (norm_vec->points[i].timestamp - curr_frame.timeStart) /
                                           (curr_frame.timeEnd - curr_frame.timeStart);

                            V3D raw_point(curr_frame.cloud_ori_downsample->points[i].x, curr_frame.cloud_ori_downsample->points[i].y,
                                          curr_frame.cloud_ori_downsample->points[i].z);

                            double weight = 1.0;
                            // add residualblock
                            //  ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<CTFunctor,
                            //                                                          1, Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters>(
                            //                                                              new CTFunctor(alpha,raw_point, pabcd, weight)
                            //                                                          );
                            //  problem.AddResidualBlock(cost_function,
                            //                          loss_function,
                            //                          begin_pose.data(),end_pose.data()
                            //                          );
                            //  ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<point2planeFunction,
                            //                                                          1, Sophus::SE3d::num_parameters>(
                            //                                                              new point2planeFunction(raw_point, pabcd, weight)
                            //                                                          );
                            //  problem.AddResidualBlock(cost_function,
                            //                          loss_function,
                            //                          begin_pose.data()
                            //                          );

                            // quat+trans
                            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor, 1, 4, 4, 3, 3>(
                                    new CTFunctor(alpha, raw_point, pabcd, weight));

                            problem.AddResidualBlock(cost_function,
                                                     loss_function,
                                                     begin_quat.coeffs().data(), end_quat.coeffs().data(), begin_trans.data(), end_trans.data());
                        }
                    }
                }

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


                curr_frame.setMotion(begin_quat, end_quat, begin_trans, end_trans);
                curr_frame.normalize();

                //curr_frame.update();
                curr_frame.updateFromDownSample();
                if(curr_frame.pose.compareDiff(pre_pose)) {
                    pre_pose = curr_frame.pose;
                    break;
                }

                pre_pose = curr_frame.pose;


            }

            // update local kd_map
            kd_map.updateLocalMap(curr_frame, neighbor);
            //updateLocalMap(curr_frame,neighbor);
            ROS_INFO("kd_map size:%d",kd_map.local_ikdtree.size());
            *globalMap += *curr_frame.cloud_world;

            // publish map
            publish();
            // debugPrint();
            r.sleep();
        }
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


        sensor_msgs::PointCloud2 downsample_cloud;
        pcl::toROSMsg(*curr_frame.cloud_ori_downsample,downsample_cloud);
        downsample_cloud.header.stamp = ros::Time(headertime);
        downsample_cloud.header.frame_id= "map";//for debug

        geometry_msgs::PoseStamped geo_odometry_pose;
        geo_odometry_pose.header = odometry_pose.header;
        geo_odometry_pose.pose = odometry_pose.pose.pose;

        globalPath.header.stamp = ros::Time(headertime);
        globalPath.header.frame_id = "map";
        globalPath.poses.push_back(geo_odometry_pose);
        traj_pub.publish(globalPath);

        cloud_downsample.publish(downsample_cloud);

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