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

struct LocationConsistency{
    LocationConsistency(const Eigen::Vector3d previous_location, const double beta):previous_location_(previous_location),beta_(beta){}

    template<typename  T>
    bool operator()(const T *const location_params, T* residual)const{
        residual[0]=beta_*(location_params[0]-previous_location_(0,0));
        residual[1]=beta_*(location_params[1]-previous_location_(1,0));
        residual[2]=beta_*(location_params[2]-previous_location_(2,0));
        return true;
    }

private:
    Eigen::Vector3d previous_location_;
    double beta_ =1.0;
};

struct ConstantVelocity{
    ConstantVelocity(const Eigen::Vector3d &previoud_velocity, double beta ):previou_velocity_(previoud_velocity),beta_(beta){}

    template<typename T>
    bool operator()(const T * const begin_t, const T *const end_t, T * residual) const{
        residual[0]=beta_*(end_t[0]-begin_t[0]-previou_velocity_(0,0));
        residual[1]=beta_*(end_t[1]-begin_t[1]-previou_velocity_(1,0));
        residual[2]=beta_*(end_t[2]-begin_t[2]-previou_velocity_(2,0));
        return true;
    }
private:
    Eigen::Vector3d previou_velocity_;
    double beta_=1.0;
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

//---------------------new----------------------------------
struct CTFunctor2{

    CTFunctor2(double alpha,const V3D &raw_point,const V3D &ref_points, double weight, V3D normal)
            :alpha_(alpha),raw_points_(raw_point),ref_points_(ref_points),weight_(weight),normal_(normal) {}


    template<class T>//必须使用模板
    bool operator()( const T *const begin_rot,
                     const T *const end_rot, const T * const begin_trans,  const T * const end_trans,T * residual)const{

        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_begin(const_cast<T *>(begin_trans));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_end(const_cast<T *>(end_trans));
        Eigen::Quaternion<T> quad_inter=quat_begin.normalized().slerp(T(alpha_),quat_end.normalized());

        quad_inter.normalize();

        Eigen::Matrix<T, 3, 1> tr;

        tr(0, 0) = (1-alpha_) * trans_begin(0,0) + alpha_ * trans_end(0,0);
        tr(1, 0) = (1-alpha_) * trans_begin(1,0) + alpha_ * trans_end(1,0);
        tr(2, 0) = (1-alpha_) * trans_begin(2,0) + alpha_ * trans_end(2,0);

        Eigen::Matrix<T,3,1> world_points= quad_inter* raw_points_.template cast<T>() + tr;

        T product = ( ref_points_-world_points.template cast<T>()).transpose() * normal_.template cast<T>();
        residual[0] = T(weight_) * product;



        return true;
    }

    double alpha_;
    V3D raw_points_;
    V3D ref_points_;
    Eigen::Vector3d  normal_;
    double weight_;
    //FunctorT func;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


