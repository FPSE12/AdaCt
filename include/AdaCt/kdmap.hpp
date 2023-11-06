#pragma once

#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/utility.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "AdaCt/frame_info.hpp"

// struct Box
// {
//     float vertex_min[3];
//     float vertex_max[3];
// };

#define num_neighbor 5

// template class KD_TREE<PointType>;

class kdMap
{
public:
    KD_TREE<pcl::PointXYZ> local_ikdtree; // faster

    double map_resolution;

    bool initial;

    BoxPointType local_map;
    double local_map_size;

    const float MOV_THRESHOLD = 1.5f;
    float DET_RANGE = 300.0f;
    std::vector<BoxPointType> localcub_needrm;

//    std::vector<std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>> neighbor;
    // std::vector<bool> HaveValidPlane;
    // pcl::PointCloud<PointType>::Ptr norm_vec;
    // int num_neighbor;

    kdMap()
    {
        //std::cout << "12" << std::endl;
        this->local_map_size = 1000; // 1000
        this->map_resolution = 0.5;
        initial = false;
        // num_neighbor = 5;
//        local_ikdtree.set_downsample_param(map_resolution);

        // norm_vec.reset(new pcl::PointCloud<PointType>());
    }

    // lidar in localmap center, kdtree
    void localMapInit(pcl::PointCloud<PointXYZIRT>::Ptr ori_cloud)
    {
        local_ikdtree.set_downsample_param(map_resolution);
        if (!initial)
        {
            for (int i = 0; i < 3; i++)
            {
                local_map.vertex_min[i] = 0.0 - local_map_size / 2;
                local_map.vertex_max[i] = 0.0 + local_map_size / 2;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*ori_cloud, *cloud_copy);
        local_ikdtree.Build(cloud_copy->points);
        initial = true;
        return;
    }

    template<typename T1 , typename T2>
    double point_dis(T1 p1, T2 p2)
    {
        double d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return d;
    }

    void delete_margincube(V3D laser_world_pose)
    {
        float dist_to_map_edge[3][2];
        bool needmove = false;

        localcub_needrm.clear();

        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(laser_world_pose(i) - local_map.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(laser_world_pose(i) - local_map.vertex_max[i]);

            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            {
                ROS_INFO("NEED REMOVE");
                needmove = true;
            }
        }

        if (!needmove)
            return;

        BoxPointType new_local_map, temp;

        new_local_map = local_map;

        double mov_dist = max((local_map_size - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));

        for (int i = 0; i < 3; i++)
        {
            temp = local_map;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
            {
                new_local_map.vertex_max[i] -= mov_dist;
                new_local_map.vertex_min[i] -= mov_dist;
                temp.vertex_min[i] = local_map.vertex_max[i] - mov_dist;

                localcub_needrm.push_back(temp);
            }
            else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            {
                new_local_map.vertex_max[i] += mov_dist;
                new_local_map.vertex_min[i] += mov_dist;
                temp.vertex_max[i] = local_map.vertex_min[i] + mov_dist;
                localcub_needrm.push_back(temp);
            }
        }

        local_map = new_local_map;

        // delete cube
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points_history;
        local_ikdtree.acquire_removed_points(points_history);

        if (!localcub_needrm.empty() && localcub_needrm.size() > 0) {
            int delete_count = local_ikdtree.Delete_Point_Boxes(localcub_needrm);
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
                mid_point.x = floor(curr_frame.cloud_world->points[i].x / map_resolution) * map_resolution + 0.5 * map_resolution;
                mid_point.y = floor(curr_frame.cloud_world->points[i].y / map_resolution) * map_resolution + 0.5 * map_resolution;
                mid_point.z = floor(curr_frame.cloud_world->points[i].z / map_resolution) * map_resolution + 0.5 * map_resolution;

                float dist = point_dis(temp, mid_point);
                //ROS_INFO("%f", neighbor_i[0].x);
                if (fabs(neighbor_i[0].x - mid_point.x) > 0.5 * map_resolution && fabs(neighbor_i[0].y - mid_point.x) > 0.5 * map_resolution && fabs(neighbor_i[0].z - mid_point.x) > 0.5 * map_resolution)
                {
                    pointNoNeedDownSample.push_back(temp);
                    continue;
                }

                for (int j = 0; j < num_neighbor; j++)
                {
                    if (neighbor_i.size() < num_neighbor)
                        break;
                    if (point_dis(neighbor_i[j], mid_point) < dist)
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

        local_ikdtree.Add_Points(pointAdd, true);
        local_ikdtree.Add_Points(pointNoNeedDownSample, false);

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

    // void searchNeighbor(frame_info currframe, std::vector<bool> & HaveValidPlane, pcl::PointCloud<PointType>::Ptr norm_vec)
    // {
    //     neighbor.clear();

    //     std::vector<float> pointSearchSqDis(num_neighbor);

    //     int pointcloud_size = currframe.cloud_world->points.size();
    //     neighbor.resize(pointcloud_size);
    //     HaveValidPlane=std::vector<bool>(pointcloud_size,false);
    //     norm_vec->clear();
    //     norm_vec->resize(pointcloud_size);

    //     for (int i = 0; i < pointcloud_size; i++)
    //     {
    //         auto &neighbor_i = neighbor[i];

    //         local_ikdtree.Nearest_Search(currframe.cloud_world->points[i], num_neighbor, neighbor_i, pointSearchSqDis);

    //         if (neighbor_i.size() < num_neighbor || pointSearchSqDis[num_neighbor - 1] > 5)
    //         {
    //             continue;
    //         }
    //         Eigen::Vector4d pabcd;
    //         PointType point_world = currframe.cloud_world->points[i];

    //         //use ori or deskew?
    //         Eigen::Vector3d point_body (currframe.cloud_ori->points[i].x,currframe.cloud_ori->points[i].y, currframe.cloud_ori->points[i].z);
    //         if(esti_plane(pabcd, neighbor_i, 0.1f)){
    //             double point2plane= pabcd(0)* point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z+pabcd(3);
    //             double s =1-0.9 * fabs(point2plane)/sqrt(point_body.norm());

    //             if(s>0.9){
    //                 HaveValidPlane[i]=true;
    //                 norm_vec->points[i].x=pabcd(0);
    //                 norm_vec->points[i].y=pabcd(1);
    //                 norm_vec->points[i].z=pabcd(2);
    //                 norm_vec->points[i].intensity=point2plane;
    //             }
    //         };
    //     }
    //     return;
    // }

    void searchNeighbor(PointXYZIRT world_points, std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &neighbor_i, std::vector<float> &pointSearchSqDis)
    {
        // pointSearchSqDis.resize
        pcl::PointXYZ world_points_copy;
        world_points_copy.x = world_points.x;
        world_points_copy.y = world_points.y;
        world_points_copy.z = world_points.z;
       // world_points_copy.intensity = world_points.intensity;
        local_ikdtree.Nearest_Search(world_points_copy, num_neighbor, neighbor_i, pointSearchSqDis);
    }


};