#pragma once

#include "tsl/robin_map.h"
#include <glog/logging.h>
#include <optional>
#include "voxelblock.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "AdaCt/types.hpp"
#include "omp.h"


#ifdef DEBUG_PRINT
#define Debug_print true
#else
#define Debug_print false
#endif

//copy from hku_mars->voxelmap:

#define MIN_DIS_THRESHOLD 0.05

static int plane_id=0;

class OctoTree{
public:
    double voxel_center_[3];
    float quater_length_;

    std::vector<PointType> all_points_;
    std::vector<PointType> new_points_;
    int all_points_num_;
    int new_points_num_;
    int max_points_size_;//point'num > this , stop update
    int max_cov_points_size_;
    int update_size_threshold_;// new_point_num > this, && update_cov_enable_, call init_plane()
    bool init_octo_;
    bool update_cov_enable_;
    bool update_enable_;


    Plane * plane_ptr_;
    float planer_threshold_;
    int max_plane_update_threshold_;// Voxel num > this , init_plane


    int max_layer_;
    int layer_;
    std::vector<int> layer_point_size_;

    int octo_state_; //0: the voxel is not divided; 1 : divided
    OctoTree * leaves_[8];

    OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
             int max_point_size, int max_cov_points_size, float planer_threshold)
            : max_layer_(max_layer), layer_(layer),
              layer_point_size_(layer_point_size), max_points_size_(max_point_size),
              max_cov_points_size_(max_cov_points_size),
              planer_threshold_(planer_threshold) {
        all_points_.clear();
        octo_state_ = 0;
        new_points_num_ = 0;
        all_points_num_ = 0;
        // when new points num > 5, do a update
        update_size_threshold_ = 5;
        init_octo_ = false;
        update_enable_ = true;
        update_cov_enable_ = true;
        max_plane_update_threshold_ = layer_point_size_[layer_];
        for (int i = 0; i < 8; i++) {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new Plane;
    }

    Eigen::Vector3d foundCenter(const Eigen::Vector3d &center, const std::vector<PointType> &points){
        double min_dis = std::numeric_limits<double>::max();
        Eigen::Vector3d target;
        for(auto &point: points){
            double dis = (center[0]-point.point_world[0])*(center[0]-point.point_world[0])+
                    (center[1]-point.point_world[1])*(center[1]-point.point_world[1])+
                    (center[2]-point.point_world[2])*(center[2]-point.point_world[2]);
            if(dis < min_dis){
                min_dis = dis;
                target = point.point_world;
            }
        }
        return target;
    }

    bool canAddpoint2Map(const PointType & pv, const std::vector<PointType> & points, double min_dis_thres){
        for(auto &point: points){
            double dis = (pv.point_world[0]-point.point_world[0])*(pv.point_world[0]-point.point_world[0])+
                    (pv.point_world[1]-point.point_world[1])*(pv.point_world[1]-point.point_world[1])+
                    (pv.point_world[2]-point.point_world[2])*(pv.point_world[2]-point.point_world[2]);
            if(dis < min_dis_thres * min_dis_thres){
                return false;
            }
        }
        return true;
    }

    void init_plane(const std::vector<PointType> &points, Plane *plane) {
        plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
        plane->covariance = Eigen::Matrix3d::Zero();
        plane->center = Eigen::Vector3d::Zero();
        plane->normal = Eigen::Vector3d::Zero();
        plane->plane_point = Eigen::Vector3d::Zero();
        plane->points_size = points.size();
        plane->radius = 0;
        for (auto & pv : points) {
            plane->covariance += pv.point_world * pv.point_world.transpose();
            plane->center += pv.point_world;
        }
        plane->center = plane->center / plane->points_size;
        plane->plane_point = foundCenter(plane->center, points);
        plane->covariance = plane->covariance / plane->points_size -
                            plane->center * plane->center.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();//协方差矩阵的特征值
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        int evalsMid = 3 - evalsMin - evalsMax;//巧妙地得到中间大小特征值的index
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
        Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
        // plane covariance calculation
        Eigen::Matrix3d J_Q;
        J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
                1.0 / plane->points_size;//单位矩阵，(7),中心点对所有点的导数

        //计算法向量对每个点的世界坐标的导数（7）
        if (evalsReal(evalsMin) < planer_threshold_) {//平面是一小两大，符合平面要求
            //std::cout<<"plane_valid"<<std::endl;
            std::vector<int> index(points.size());
            std::vector<Eigen::Matrix<double, 6, 6>> temp_matrix(points.size());
//            for (int i = 0; i < points.size(); i++) {
//                Eigen::Matrix<double, 6, 3> J;
//                Eigen::Matrix3d F;
//                for (int m = 0; m < 3; m++) {
//                    if (m != (int)evalsMin) {
//                        Eigen::Matrix<double, 1, 3> F_m =
//                                (points[i].point_world - plane->center).transpose() /
//                                ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
//                                (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
//                                 evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
//                        F.row(m) = F_m;
//                    } else {//最小的特征值
//                        Eigen::Matrix<double, 1, 3> F_m;
//                        F_m << 0, 0, 0;
//                        F.row(m) = F_m;
//                    }
//                }
//                J.block<3, 3>(0, 0) = evecs.real() * F;
//                J.block<3, 3>(3, 0) = J_Q;
//                plane->plane_cov += J * points[i].cov * J.transpose();//(8)，协方差矩阵
//            }

            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                    evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
                    evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                    evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->A2D = (sqrt(std::abs(evalsReal(evalsMid)))- sqrt(std::abs(evalsReal(evalsMin))))/sqrt(std::abs(evalsReal(evalsMax)));
            plane->d = -(plane->normal(0) * plane->center(0) +
                         plane->normal(1) * plane->center(1) +
                         plane->normal(2) * plane->center(2));//平面方程AX+BY+CZ+1=0,如果{A,B,C}取法向量的值，那么为AX+BY+CZ+d=0（平面中心在平面上）
            plane->is_plane = true;
            if (plane->last_update_points_size == 0) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            } else if (plane->points_size - plane->last_update_points_size > 100) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            }

            if (!plane->is_init) {
                plane->id = plane_id;//static global variant
                plane_id++;
                plane->is_init = true;
            }

        } else {// not flat enough，not plane but calculate plane's params
            if (!plane->is_init) {
                plane->id = plane_id;
                plane_id++;
                plane->is_init = true;
            }
            if (plane->last_update_points_size == 0) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            } else if (plane->points_size - plane->last_update_points_size > 100) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            }
            plane->is_plane = false;
            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                    evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
                    evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                    evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->d = -(plane->normal(0) * plane->center(0) +
                         plane->normal(1) * plane->center(1) +
                         plane->normal(2) * plane->center(2));
        }
    }

    //update center && normal && covariance
    void update_plane(const  std::vector<PointType> &points, Plane *plane) {
        Eigen::Matrix3d old_covariance = plane->covariance;
        Eigen::Vector3d old_center = plane->center;
        //在原基础上继续计算
        Eigen::Matrix3d sum_ppt =
                (plane->covariance + plane->center * plane->center.transpose()) *
                plane->points_size;
        Eigen::Vector3d sum_p = plane->center * plane->points_size;
        for (auto & point  : points) {
            Eigen::Vector3d pv = point.point_world;
            sum_ppt += pv * pv.transpose();
            sum_p += pv;
        }
        plane->points_size = plane->points_size + points.size();
        plane->center = sum_p / plane->points_size;
        plane->covariance = sum_ppt / plane->points_size -
                            plane->center * plane->center.transpose();
        plane->plane_point = foundCenter(plane->center, points);
        //recalcualate normal
        Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3d::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        int evalsMid = 3 - evalsMin - evalsMax;
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
        Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
        if (evalsReal(evalsMin) < planer_threshold_) {
            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                    evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
                    evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                    evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->A2D = (evalsReal(evalsMid)-evalsReal(evalsMin))/evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->d = -(plane->normal(0) * plane->center(0) +
                         plane->normal(1) * plane->center(1) +
                         plane->normal(2) * plane->center(2));

            plane->is_plane = true;
            plane->is_update = true;
        } else {
            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                    evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
                    evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                    evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->A2D = (evalsReal(evalsMid)-evalsReal(evalsMin))/evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));//max_eigenvalue
            plane->d = -(plane->normal(0) * plane->center(0) +
                         plane->normal(1) * plane->center(1) +
                         plane->normal(2) * plane->center(2));
            plane->is_plane = false;
            plane->is_update = true;
        }
    }

    void init_octo_tree() {
        if (all_points_.size() > max_plane_update_threshold_) {//point_num
            init_plane(all_points_, plane_ptr_);
            if (plane_ptr_->is_plane == true) {//如果是平面就不继续分割
                octo_state_ = 0;
                if (all_points_.size() > max_cov_points_size_) {
                    update_cov_enable_ = false;
                }
                if (all_points_.size() > max_points_size_) {
                    update_enable_ = false;
                }
            } else {//否则就要继续分割，状态为1
                octo_state_ = 1;
                cut_octo_tree();
            }
            init_octo_ = true;
            new_points_num_ = 0;
            //      all_points_.clear();
        }
    }

    void cut_octo_tree() {
        if (layer_ >= max_layer_) {
            octo_state_ = 0;
            return;
        }
        //ensure subvoxel index
        for (auto & point : all_points_) {
            int xyz[3] = {0, 0, 0};
            if (point.point_world[0] > voxel_center_[0]) {
                xyz[0] = 1;
            }
            if (point.point_world[1] > voxel_center_[1]) {
                xyz[1] = 1;
            }
            if (point.point_world[2] > voxel_center_[2]) {
                xyz[2] = 1;
            }
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
            if (leaves_[leafnum] == nullptr) {
                leaves_[leafnum] = new OctoTree(
                        max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                        max_cov_points_size_, planer_threshold_);
                leaves_[leafnum]->voxel_center_[0] =
                        voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                leaves_[leafnum]->voxel_center_[1] =
                        voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                leaves_[leafnum]->voxel_center_[2] =
                        voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            }
            leaves_[leafnum]->all_points_.emplace_back(point);
            //leaves_[leafnum]->new_points_num_++;
        }

        std::vector<PointType>().swap(all_points_);//if cut clear all_points because in subvoxel
        //all_points_.clear();
        for (uint i = 0; i < 8; i++) {//继续对子voxel判断是否拟合
            if (leaves_[i] != nullptr) {
                if (leaves_[i]->all_points_.size() >
                    leaves_[i]->max_plane_update_threshold_) {
                    init_plane(leaves_[i]->all_points_, leaves_[i]->plane_ptr_);
                    if (leaves_[i]->plane_ptr_->is_plane) {
                        leaves_[i]->octo_state_ = 0;
                    } else {
                        leaves_[i]->octo_state_ = 1;
                        leaves_[i]->cut_octo_tree();
                    }
                    leaves_[i]->init_octo_ = true;
                    leaves_[i]->new_points_num_ = 0;
                }
            }
        }
    }

    void UpdateOctoTree(const PointType &pv) {
        if (!init_octo_) {//not init
            new_points_num_++;
            all_points_num_++;
            all_points_.push_back(pv);
            if (all_points_.size() > max_plane_update_threshold_) {//没有更新的情况下

                init_octo_tree();
            }
        } else {
            if (plane_ptr_->is_plane ) {// is plane
//                if(!canAddpoint2Map(pv,all_points_,MIN_DIS_THRESHOLD)){
//                    return ;
//                }
                if (update_enable_) {// point_num not bigger than threshold
                    new_points_num_++;
                    all_points_num_++;
                    if (update_cov_enable_) {//更新cov
                        all_points_.push_back(pv);
                    } else {
                        new_points_.push_back(pv);
                    }
                    if (new_points_num_ > update_size_threshold_) {//每增加一定的点，就继续更新平面
                        if (update_cov_enable_) {
                            init_plane(all_points_, plane_ptr_);//重新计算
                        }
                        new_points_num_ = 0;//new point 置0
                        new_points_.clear();
                    }
                    if (all_points_num_ >= max_cov_points_size_) {//bigger, make zero
                        update_cov_enable_ = false;
//                        std::vector<PointType>().swap(all_points_);

                    }
                    if (all_points_num_ >= max_points_size_) {
                        update_enable_ = false;
                        plane_ptr_->update_enable = false;
                        std::vector<PointType>().swap(new_points_);
                    }
                } else {
                    return;
                }
            } else {// not plane
                if (layer_ < max_layer_) {//如果还没到最小分辨率
//                    if (all_points_.size() != 0) {//clear use the sub_voxel
////                        std::vector<PointType>().swap(all_points_);//deleta highlevel's all_point(not plane)
//                        cut_octo_tree();//clear all points
//                    }
                    if (new_points_.size() != 0) {
                        std::vector<PointType>().swap(new_points_);
                    }
                    int xyz[3] = {0, 0, 0};
                    if (pv.point_world[0] > voxel_center_[0]) {
                        xyz[0] = 1;
                    }
                    if (pv.point_world[1] > voxel_center_[1]) {
                        xyz[1] = 1;
                    }
                    if (pv.point_world[2] > voxel_center_[2]) {
                        xyz[2] = 1;
                    }
                    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                    if (leaves_[leafnum] != nullptr) {
                        leaves_[leafnum]->UpdateOctoTree(pv);//在sub voxel更新， 递归
                    } else {
                        leaves_[leafnum] = new OctoTree(
                                max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                                max_cov_points_size_, planer_threshold_);
                        leaves_[leafnum]->layer_point_size_ = layer_point_size_;
                        leaves_[leafnum]->voxel_center_[0] =
                                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                        leaves_[leafnum]->voxel_center_[1] =
                                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                        leaves_[leafnum]->voxel_center_[2] =
                                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                        leaves_[leafnum]->UpdateOctoTree(pv);
                    }
                } else {//到了最小分辨率，只能在这个分辨率上更新了
                    if (update_enable_) {
                        new_points_num_++;
                        all_points_num_++;
                        if (update_cov_enable_) {//可以更新，就用所有的点重新拟合平面
                            all_points_.push_back(pv);
                        } else {
                            new_points_.push_back(pv);//点已经有很多了，就用新的点拟合平面
                        }
                        if (new_points_num_ > update_size_threshold_) {
                            if (update_cov_enable_) {
                                init_plane(all_points_, plane_ptr_);
                            } else {
                                update_plane(new_points_, plane_ptr_);
                                new_points_.clear();
                            }
                            new_points_num_ = 0;
                        }
                        if (all_points_num_ >= max_cov_points_size_) {
                            update_cov_enable_ = false;
//                            std::vector<PointType>().swap(all_points_);
                        }
                        if (all_points_num_ >= max_points_size_) {
                            update_enable_ = false;
                            plane_ptr_->update_enable = false;
                            std::vector<PointType>().swap(new_points_);
                        }
                    }
                }
            }
        }
    }

    void GetPclCloud(pcl::PointCloud<PointXYZIRT>::Ptr PclCloud){
        if(all_points_.empty()){
            for(auto leave : leaves_){
                if(leave!= nullptr){
                    leave->GetPclCloud(PclCloud);
                }
            }
            return;
        }

        for(auto & p : all_points_){
            PointXYZIRT pc;
            pc.x = p.point_world[0];
            pc.y = p.point_world[1];
            pc.z = p.point_world[2];
            pc.timestamp = p.timestamp;
            pc.intensity = p.intensity;
            PclCloud->push_back(pc);
        }
    }
};


void buildVoxelMap(const std::vector<PointType> &input_points,
                   const float voxel_size, const int max_layer,
                   const std::vector<int> &layer_point_size,
                   const int max_points_size, const int max_cov_points_size,
                   const float planer_threshold,
                   tsl::robin_map<Voxel, OctoTree *> &feat_map) {
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const PointType p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;//-1
            }
        }
        Voxel position((int)loc_xyz[0], (int)loc_xyz[1],
                           (int)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            feat_map[position]->all_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
        } else {
            OctoTree *octo_tree =
                    new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                                 max_cov_points_size, planer_threshold);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;//1/4Voxelsize
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->all_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
            feat_map[position]->layer_point_size_ = layer_point_size;
        }
    }
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
        iter->second->init_octo_tree();//init plane
    }
    //std::cout<<"all gird_num:"<<feat_map.size()<<std::endl;
    //std::cout<<"build_plane_num: "<<plane_id<<std::endl;
}


void updateVoxelMap(const std::vector<PointType> &input_points,
                    const float voxel_size, const int max_layer,
                    const std::vector<int> &layer_point_size,
                    const int max_points_size, const int max_cov_points_size,
                    const float planer_threshold,
                    tsl::robin_map<Voxel, OctoTree *> &feat_map) {
    uint plsize = input_points.size();

    for(auto & p_v : input_points){
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        Voxel position((int)loc_xyz[0], (int)loc_xyz[1],
                           (int)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            feat_map[position]->UpdateOctoTree(p_v);
        } else {
            OctoTree *octo_tree =
                    new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                                 max_cov_points_size, planer_threshold);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->UpdateOctoTree(p_v);
        }
    }

    if(Debug_print){
        std::cout<<"plane_num: "<<plane_id<<std::endl;
    }

}


void getVoxelMap(const tsl::robin_map<Voxel, OctoTree*> & feat_map , pcl::PointCloud<PointXYZIRT>::Ptr pclCloud){
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
        iter->second->GetPclCloud(pclCloud);//init plane
    }
}



void build_single_residual(const PointType &pv, const OctoTree *current_octo,
                           const int current_layer, const int max_layer,
                           const double sigma_num, bool &is_sucess,
                           double &distance, ptpl &single_ptpl) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_world;
    if (current_octo->plane_ptr_->is_plane) {//如果是个平面
        Plane &plane = *current_octo->plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.center;
        double proj_x = p_world_to_center.dot(plane.x_normal);
        double proj_y = p_world_to_center.dot(plane.y_normal);
        float dis_to_plane =
                fabs(plane.normal[0] * p_w[0] + plane.normal[1] * p_w[1] +
                     plane.normal[2] * p_w[2] + plane.d);
//        float dis_to_center =
//                (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
//                (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
//                (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
        //use the real point is better
        float dis_to_center =
                (plane.plane_point(0) - p_w(0)) * (plane.plane_point(0) - p_w(0)) +
                (plane.plane_point(1) - p_w(1)) * (plane.plane_point(1) - p_w(1)) +
                (plane.plane_point(2) - p_w(2)) * (plane.plane_point(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
        if (range_dis <= radius_k * plane.radius) {
//            Eigen::Matrix<double, 1, 6> J_nq;
//            J_nq.block<1, 3>(0, 0) = p_w - plane.center;
//            J_nq.block<1, 3>(0, 3) = -plane.normal;//J_wi前两部分，公式(11)
//            double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();//公式（10）
//            sigma_l += plane.normal.transpose() * pv.cov * plane.normal;//
//            if (dis_to_plane < sigma_num * sqrt(sigma_l)) {//点的残差在3 theta之内
//                is_sucess = true;
//                double this_prob = 1.0 / (sqrt(sigma_l)) *
//                                   exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);//高斯分布公式，在dis_to_plane处的概率值
//                if (this_prob > prob) {
//                    prob = this_prob;
//                    single_ptpl.point = pv.point;
//                    single_ptpl.plane_cov = plane.plane_cov;
//                    single_ptpl.normal = plane.normal;
//                    single_ptpl.center = plane.center;
//                    single_ptpl.d = plane.d;
//                    single_ptpl.layer = current_layer;
//                    single_ptpl.point_alpha = pv.alpha;
//                    single_ptpl.A2D = plane.A2D;
//                }
//                return;
//            } else {
//                // is_sucess = false;
//                return;
//            }


            is_sucess = true;
            if(dis_to_plane < distance){
                single_ptpl.point = pv.point;
                single_ptpl.point_world = pv.point_world;
                single_ptpl.plane_cov = plane.plane_cov;
                single_ptpl.normal = plane.normal;
                single_ptpl.center = plane.center;
                single_ptpl.d = plane.d;
                single_ptpl.layer = current_layer;
                single_ptpl.point_alpha = pv.alpha;
                single_ptpl.A2D = plane.A2D;
                single_ptpl.plane_point = plane.plane_point;
            }
            return;
        }else{

//            if(Debug_print){
//                std::cout<<"rang too far!"<<"range:"<<range_dis<<"plane_radius:"<<plane.radius<<std::endl;
//            }
            is_sucess = false;
            return;
        }

    } else {//如果当前不是平面，对其子Voxel继续进行
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {

                    OctoTree *leaf_octo = current_octo->leaves_[leafnum];
                    build_single_residual(pv, leaf_octo, current_layer + 1, max_layer,
                                          sigma_num, is_sucess, distance, single_ptpl);
                }
            }
            return;
        } else {
             is_sucess = false;
            return;
        }
    }

}


void BuildResidualListOMP(const tsl::robin_map<Voxel, OctoTree *> &voxel_map,
                          const double voxel_size, const double sigma_num,
                          const int max_layer,
                          const std::vector<PointType> &pv_list,
                          std::vector<ptpl> &ptpl_list,
                          std::vector<Eigen::Vector3d> &non_match) {
    std::mutex mylock;
    ptpl_list.clear();
    std::vector<ptpl> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size());
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
        useful_ptpl[i] = false;
    }
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < index.size(); i++) {
        PointType pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        Voxel position((int)loc_xyz[0], (int)loc_xyz[1],
                           (int)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            OctoTree *current_octo = iter->second;
            ptpl single_ptpl;
            bool is_sucess = false;
            double prob = 1000.0;//残差的不确定性（以残差的协方差（一维）为方差的高斯分布中的概率）
            build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                                  is_sucess, prob, single_ptpl);
            if (!is_sucess) {//如果没有找到
                Voxel near_position = position;
                if (loc_xyz[0] >
                    (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                                         current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] >
                    (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                                         current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] >
                    (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                                         current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find(near_position);//找最近的再次寻找
                if (iter_near != voxel_map.end()) {
                    build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                                          is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {//成功匹配

                mylock.lock();
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
                mylock.unlock();
            } else {//没有匹配
                mylock.lock();
                useful_ptpl[i] = false;
                mylock.unlock();
            }
        }
    }
    for (size_t i = 0; i < useful_ptpl.size(); i++) {
        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        }
    }
}
