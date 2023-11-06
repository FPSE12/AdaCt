#pragma once

#include "AdaCt/optPose.hpp"
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>
#include <Eigen/Dense>


struct point2planeFunction
{
    /* data */
    point2planeFunction(const V3D &raw, const Eigen::Vector4d &pabcd,
                        double weight) : raw_points_(raw), pabcd_(pabcd), weight_(weight) {}
    
    template<class T>
    bool operator()(const T * const linearInplot_pose_s, T * residual) const{
        Eigen::Map<Sophus::SE3<T> const > const linearInplot_pose(linearInplot_pose_s);
        V3D world_points_ =linearInplot_pose * raw_points_;
        residual[0] = weight_*((0)*pabcd_(0)+world_points_(1)*pabcd_(1)+world_points_(2)*pabcd_(2)+pabcd_(3));
        return true;

    }

    // T linearInplote_pose_;
    V3D raw_points_;
    Eigen::Vector4d pabcd_;
    double weight_;
};

struct CTFunctor{
    
    CTFunctor(double alpha,const V3D &raw_point,const Eigen::Vector4d& pabcd, double weight)
    :alpha_(alpha),raw_points_(raw_point),pabcd_(pabcd),weight_(weight){}


    template<class T>//必须使用模板
    bool operator()( const T *const begin_rot,
                    const T *const end_rot, const T * const begin_trans,  const T * const end_trans,T * residual)const{

        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_begin(const_cast<T *>(begin_trans));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_end(const_cast<T *>(end_trans));
        Eigen::Quaternion<T> quad_inter=quat_begin.normalized().slerp(T(alpha_),quat_end.normalized());

        Eigen::Matrix<T, 3, 1> tr;

        tr(0, 0) = (1-alpha_) * trans_begin(0,0) + alpha_ * trans_end(0,0);
        tr(1, 0) = (1-alpha_) * trans_begin(1,0) + alpha_ * trans_end(1,0);
        tr(2, 0) = (1-alpha_) * trans_begin(2,0) + alpha_ * trans_end(2,0);

        Eigen::Matrix<T,3,1> world_points= quad_inter.normalized() * raw_points_.template cast<T>() + tr;

        residual[0] =T( weight_) *(world_points(0,0) *pabcd_(0) + world_points(1,0) *pabcd_(1) + world_points(2,0) *pabcd_(2) + pabcd_(3));



        return true;
    }

    double alpha_;
    V3D raw_points_;
    Eigen::Vector4d pabcd_;
    double weight_;
    //FunctorT func;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


