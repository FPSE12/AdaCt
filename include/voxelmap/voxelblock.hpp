#pragma once

#include <iostream>
#include <vector>
#include <queue>

#include <Eigen/Core>

#include "voxel.hpp"


template<typename T>
double PointDis(T p1, T p2){
    return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
}

//------------------------------------VoxelblockDescription----------------------------

enum VOXELBLOCK_PROPERTIES {
    NO_VALUES = 0,
    LINE = 1 << 0,
    NORMAL = 1 << 1,
    PLANARITY = 1 << 2,
    LINEARITY = 1 << 3,
    COV_DET = 1 << 4,
    KDTREE = 1 << 5,
    A2D = 1 << 6,
    ALL = LINE | NORMAL | PLANARITY | LINEARITY | COV_DET | KDTREE | A2D,
    ALL_BUT_KDTREE = LINE | NORMAL | PLANARITY | LINEARITY | COV_DET | A2D
};

template<typename T>
struct VoxelBlockDescription{
    T planarity = T(-1.0);

    T linearity = T(-1.0);

    T a2D = T(-1);//define the planartity

    T cov_det = T(-1.0); // Determinant of the covariance

    Eigen::Matrix<T, 3, 1> line;

    Eigen::Matrix<T, 3, 1> normal;

    Eigen::Matrix<T, 3, 1> barycenter;

    Eigen::Matrix<T, 3, 3> covariance;
};


template<typename T>
VoxelBlockDescription<T> ComputeNeighborhoodInfo(const Eigen::Matrix<T,3,1> &barycenter,
                                                 const Eigen::Matrix<T, 3, 3> &covariance, int values){
    VoxelBlockDescription<T> result;

    result.barycenter = barycenter;
    result.covariance = covariance;

    Eigen::JacobiSVD<Eigen::Matrix<T,3,3>> svd(covariance,Eigen::ComputeFullV);


    if (values & LINE) {
        Eigen::Matrix<T,3,3> V = svd.matrixV();
        result.line = V.template block<3, 1>(0, 0);
    }

    if (values & NORMAL) {
        Eigen::Matrix<T,3,3> V = svd.matrixV();
        result.normal = V.template block<3, 1>(0, 2);
    }
    Eigen::Matrix<T, 3, 1> singular_values = svd.singularValues().cwiseAbs();
    if (values & LINEARITY)
        result.linearity = (singular_values[0] - singular_values[1]) / singular_values[0];
    if (values & PLANARITY)
        result.planarity = (singular_values[1] - singular_values[2]) / singular_values[0];
    if (values & A2D)
        result.a2D = (std::sqrt(singular_values[1]) - std::sqrt(singular_values[2])) / std::sqrt(
                singular_values[0]); //Be careful, the eigenvalues are not correct with the iterative way to compute the covariance matrix
    if (values & COV_DET)
        result.cov_det = covariance.determinant();

    return result;
}







//--------------------------------------------Voxelblock-----------------------------------
template<class PointT>
struct VoxelBlock{


    explicit VoxelBlock(int num_points=20):num_points_(num_points){
        points.reserve(num_points);
        normals.reserve(num_points);
        frame_ids.reserve(num_points);
        point_ids.reserve(num_points);
//        is_normal_oriented.resize(false);
//        is_normal_computed.resize(false);
        is_normal_computed.reserve(num_points);
        is_normal_oriented.reserve(num_points);

//        is_normal_computed.reserve(num_points);
//        is_normal_oriented.reserve(num_points);
        // timestamps.reserve(num_points);
        mid_point.x=0;
        mid_point.y=0;
        mid_point.z=0;
    }



    void calculateMid(PointT newP){
        mid_point.x = (mid_point.x * points.size()+newP.x)/(points.size()+1);
        mid_point.y = (mid_point.y * points.size()+newP.y)/(points.size()+1);
        mid_point.z = (mid_point.z * points.size()+newP.z)/(points.size()+1);
    }

    PointT findCloseToMid() const {
        double min_dis = std::numeric_limits<double>::max();
        PointT target_min;
        for(int i=0;i<points.size();i++){
            auto point=points[i];
            double dis = PointDis(point ,mid_point);

            if(dis < min_dis){
                target_min = point;
                min_dis = dis;

            }
        }
        return target_min;
    }

    void addPoint(const PointT &point){
        if(points.size()>=num_points_){
            //ROS_INFO("full!");
            //return;
            //vec : double
            num_points_=num_points_+num_points_;

        }

        calculateMid(point);
        points.push_back(point);

//        if( !legal()) {
//            ROS_ERROR("VOXEL BLOCK INSERT ERROR!");
//        }
        return;
    }


    void addPointWithProperties(const PointT &point, int frame_id=0, int point_id=0){
        if(points.size()>=num_points_){
            //ROS_INFO("full!");
            //return;
            //vec : double
            num_points_=num_points_+num_points_;

        }
        //mid_point = (mid_point * points.size() + point)/(points.size()+1);
        //calculateMid(point);
        points.push_back(point);
        frame_ids.push_back(frame_id);
        point_ids.push_back(point_id);
        is_normal_oriented.push_back(false);
        is_normal_computed.push_back(false);
//        if( !legal()) {
//            ROS_ERROR("VOXEL BLOCK INSERT ERROR!");
//        }
        return;
    }

    void computeDescription(int values){
        if(points.size()<MinValidNeighborSize()){
            isValid= false;
            return;
        }
        Eigen::Vector3d braycenter = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

        Eigen::Vector3d point_ref;

        for(auto &point:points){
            point_ref = getPointXYZ(point);
            braycenter += point_ref;
            cov += (point_ref*point_ref.transpose());
        }

        braycenter /= (double) points.size();
        cov /= (double) points.size();

        cov -=braycenter* braycenter.transpose();

        description = ComputeNeighborhoodInfo(braycenter,cov,values);

        computed_values = values;

        //build kdtree?

        isValid=true;
    }



    inline Eigen::Vector3d getPointXYZ(PointT point){
        Eigen::Vector3d T(point.x,point.y,point.z);
        return T;
    }

    static int MinValidNeighborSize(){
        return 5;
    }

    inline bool isFull() const {return num_points_ == points.size();}
    inline int numPoints() const{
        return points.size();
    }
    inline int Capacity(){
        return num_points_;
    }


    int num_points_;
    PointT mid_point;
    VoxelBlockDescription<double> description;

    //points properties
    std::vector<PointT> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<int> frame_ids;
    std::vector<int> point_ids;
    std::vector<bool> is_normal_computed;
    std::vector<bool> is_normal_oriented;

    bool isValid=false;
    int computed_values;
};


//-------------------------------for neighbor-----------------------------
using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

struct __Comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
        return std::get<0>(left) < std::get<0>(right);//big to small
    }
};

typedef  std::priority_queue<pair_distance_t,std::vector<pair_distance_t>, __Comparator> Neighbors_queue;


