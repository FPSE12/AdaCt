#include "AdaCt/utility.h"
//#include "AdaCt/frame_info.hpp"
#include "AdaCt/kdmap.hpp"
//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"
//
//kdMap have to put before main()
kdMap kd_map;
//KD_TREE<pcl::PointXYZ> ikd_tree;

//---------------------------------------------------------------------------------------
void localMapInit(pcl::PointCloud<PointXYZIRT>::Ptr ori_cloud)
{
    kd_map.local_ikdtree.set_downsample_param(kd_map.map_resolution);
    if (!kd_map.initial)
    {
        for (int i = 0; i < 3; i++)
        {
            kd_map.local_map.vertex_min[i] = 0.0 - kd_map.local_map_size / 2;
            kd_map.local_map.vertex_max[i] = 0.0 + kd_map.local_map_size / 2;
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*ori_cloud, *cloud_copy);
    kd_map.local_ikdtree.Build(cloud_copy->points);
    kd_map.initial = true;
    return;
}

void delete_margincube(V3D laser_world_pose)
{
    float dist_to_map_edge[3][2];
    bool needmove = false;

    kd_map.localcub_needrm.clear();

    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(laser_world_pose(i) - kd_map.local_map.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(laser_world_pose(i) - kd_map.local_map.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= kd_map.MOV_THRESHOLD * kd_map.DET_RANGE || dist_to_map_edge[i][1] <= kd_map.MOV_THRESHOLD * kd_map.DET_RANGE)
        {
            ROS_INFO("NEED REMOVE");
            needmove = true;
        }
    }

    if (!needmove)
        return;

    BoxPointType new_local_map, temp;

    new_local_map = kd_map.local_map;

    double mov_dist = max((kd_map.local_map_size - 2.0 * kd_map.MOV_THRESHOLD * kd_map.DET_RANGE) * 0.5 * 0.9, double(kd_map.DET_RANGE * (kd_map.MOV_THRESHOLD - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp = kd_map.local_map;
        if (dist_to_map_edge[i][0] <= kd_map.MOV_THRESHOLD * kd_map.DET_RANGE)
        {
            new_local_map.vertex_max[i] -= mov_dist;
            new_local_map.vertex_min[i] -= mov_dist;
            temp.vertex_min[i] = kd_map.local_map.vertex_max[i] - mov_dist;

            kd_map.localcub_needrm.push_back(temp);
        }
        else if (dist_to_map_edge[i][1] <= kd_map.MOV_THRESHOLD * kd_map.DET_RANGE)
        {
            new_local_map.vertex_max[i] += mov_dist;
            new_local_map.vertex_min[i] += mov_dist;
            temp.vertex_max[i] = kd_map.local_map.vertex_min[i] + mov_dist;
            kd_map.localcub_needrm.push_back(temp);
        }
    }

    kd_map.local_map = new_local_map;

    // delete cube
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points_history;
    kd_map.local_ikdtree.acquire_removed_points(points_history);

    if (!kd_map.localcub_needrm.empty() && kd_map.localcub_needrm.size() > 0) {
        int delete_count = kd_map.local_ikdtree.Delete_Point_Boxes(kd_map.localcub_needrm);
    }
    return;
}

void add_newpoint(frame_info curr_frame, std::vector<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>> neighbor)
{
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>());
//        pcl::copyPointCloud(*curr_frame.cloud_world, *cloud_copy);

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointAdd;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointNoNeedDownSample;

    int pointcloud_size = curr_frame.cloud_world->points.size();

    pointAdd.reserve(pointcloud_size);
    pointNoNeedDownSample.reserve(pointcloud_size);

    for (int i = 0; i < pointcloud_size; i++)
    {
        pcl::PointXYZ temp(curr_frame.cloud_world->points[i].x,curr_frame.cloud_world->points[i].y,curr_frame.cloud_world->points[i].z);
        if (!neighbor[i].empty())
        {
            const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> neighbor_i = neighbor[i];
            bool need_add = true;
            pcl::PointXYZ mid_point;
            mid_point.x = floor(curr_frame.cloud_world->points[i].x / kd_map.map_resolution) * kd_map.map_resolution + 0.5 * kd_map.map_resolution;
            mid_point.y = floor(curr_frame.cloud_world->points[i].y / kd_map.map_resolution) * kd_map.map_resolution + 0.5 * kd_map.map_resolution;
            mid_point.z = floor(curr_frame.cloud_world->points[i].z / kd_map.map_resolution) * kd_map.map_resolution + 0.5 * kd_map.map_resolution;

            float dist = kd_map.point_dis(temp, mid_point);
            //ROS_INFO("%f", neighbor_i[0].x);
            if (fabs(neighbor_i[0].x - mid_point.x) > 0.5 * kd_map.map_resolution && fabs(neighbor_i[0].y - mid_point.x) > 0.5 * kd_map.map_resolution && fabs(neighbor_i[0].z - mid_point.x) > 0.5 * kd_map.map_resolution)
            {
                pointNoNeedDownSample.push_back(temp);
                continue;
            }

            for (int j = 0; j < num_neighbor; j++)
            {
                if (neighbor_i.size() < num_neighbor)
                    break;
                if (kd_map.point_dis(neighbor_i[j], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                pointAdd.push_back(temp);
        }
        else
        {
            pointAdd.push_back(temp);
        }
    }

    kd_map.local_ikdtree.Add_Points(pointAdd, true);
    kd_map.local_ikdtree.Add_Points(pointNoNeedDownSample, false);

    return;
}

void updateLocalMap(frame_info currframe, std::vector<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>> neighbor)
{

    V3D curr_pose = currframe.getEndTrans();

    // if (!initial)
    // {
    //     local_ikdtree.Build(currframe.cloud_world->points);
    //     for (int i = 0; i < 3; i++)
    //     {
    //         local_map.vertex_min[i] = curr_pose(i) - local_map_size / 2;
    //         local_map.vertex_max[i] = curr_pose(i) + local_map_size / 2;
    //     }
    //     initial = true;
    //     return;
    // }

    delete_margincube(curr_pose);

    add_newpoint(currframe, neighbor); // mapping incrememtal,需要neighbor的信息
}
//----------------------------------------------------------------------------------------

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
            neighbor.resize(curr_frame.cloud_ori->size());

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
                int cloud_size = curr_frame.cloud_ori->size();
                int valid_size=0;
                norm_vec->clear();
                norm_vec->resize(cloud_size);
                // int num_neighbor=5;
                auto findNeighbor_start = std::chrono::steady_clock::now();
                for (int i = 0; i < cloud_size; i++) {
                    std::vector<float> pointSearchSqDis(num_neighbor);

                    int pointcloud_size = curr_frame.cloud_ori->points.size();
                    HaveValidPlane = std::vector<bool>(pointcloud_size, false);

                    kd_map.searchNeighbor(curr_frame.cloud_world->points[i], neighbor[i], pointSearchSqDis);

                    if (neighbor[i].size() < num_neighbor || pointSearchSqDis[num_neighbor - 1] > 5) {

                        continue;
                    }
                    Eigen::Vector4d pabcd;
                    PointXYZIRT point_world = curr_frame.cloud_world->points[i];

                    // use ori or deskew?
                    Eigen::Vector3d point_body(curr_frame.cloud_deskew->points[i].x, curr_frame.cloud_deskew->points[i].y,
                                               curr_frame.cloud_deskew->points[i].z);
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

                            V3D raw_point(curr_frame.cloud_ori->points[i].x, curr_frame.cloud_ori->points[i].y,
                                          curr_frame.cloud_ori->points[i].z);

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

                curr_frame.update();

                if(curr_frame.pose.compareDiff(pre_pose)) {
                    pre_pose = curr_frame.pose;
                    break;
                }

                pre_pose = curr_frame.pose;


            }

            // update local kd_map
//            kd_map.updateLocalMap(curr_frame, neighbor);
            updateLocalMap(curr_frame,neighbor);
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