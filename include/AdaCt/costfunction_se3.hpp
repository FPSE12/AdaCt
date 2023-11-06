#pragma once

#include "AdaCt/optPose.hpp"
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>
#include <Eigen/Dense>


// struct point2planeFunction
// {
//     /* data */
//     point2planeFunction(const V3D &raw, const Eigen::Vector4d &pabcd,
//                         double weight) : raw_points_(raw), pabcd_(pabcd), weight_(weight) {}
    
//     template<class T>
//     bool operator()(const T * const linearInplot_pose_s, T * residual) const{
//         Eigen::Map<Sophus::SE3<T> const > const linearInplot_pose(linearInplot_pose_s);
//         V3D world_points_ =linearInplot_pose * raw_points_;
//         residual[0] = weight_*((0)*pabcd_(0)+world_points_(1)*pabcd_(1)+world_points_(2)*pabcd_(2)+pabcd_(3));
//         return true;

//     }

//     // T linearInplote_pose_;
//     V3D raw_points_;
//     Eigen::Vector4d pabcd_;
//     double weight_;
// };

// struct CTFunctor{
    
//     CTFunctor(double alpha,const V3D &raw_point,const Eigen::Vector4d& pabcd, double weight)
//     :alpha_(alpha),raw_points_(raw_point),pabcd_(pabcd),weight_(weight){}


//     template<class T>//必须使用模板
//     bool operator()( const T *const begin_pose_s,  const T * const end_pose_s, T * residual)const{

//         Eigen::Map<Sophus::SE3<T> const> const begin_pose(begin_pose_s);
//         Eigen::Map<Sophus::SE3<T> const> const end_pose(end_pose_s);

//         Sophus::SE3<T> begin_pose_inv=begin_pose.inverse();
//         Sophus::SE3<T> delta=begin_pose_inv * end_pose;
//         Sophus::Vector6<T> alpha_delta_se3=alpha_*delta.log();
//         Sophus::SE3<T> linearInplot_pose=begin_pose * (SE3::exp(alpha_delta_se3));
        
//         V3D world_points_ =linearInplot_pose * raw_points_;
//         residual[0] = world_points_(0)*pabcd_(0)+world_points_(1)*pabcd_(1)+world_points_(2)*pabcd_(2)+pabcd_(3);
//         residual[0] = residual[0] * weight_;
//         return true;
//     }

//     double alpha_;
//     V3D raw_points_;
//     Eigen::Vector4d pabcd_;
//     double weight_;
//     //FunctorT func;

//    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

// // se3的参数化
// // https://www.guyuehome.com/36850
// class LocalParameterizationSE3 : public ceres::LocalParameterization
// {
// public:
//     virtual bool Plus(double const* T_raw, double const* delta_raw,
//                     double* T_plus_delta_raw) const {
//     Eigen::Map<Sophus::SE3d const> const T(T_raw);
//     // Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
//     Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);

//     const Eigen::Map<const Eigen::Vector3d> delta_phi(delta_raw);
//     const Eigen::Map<const Eigen::Vector3d> delta_rho(delta_raw+3);

//     Eigen::Matrix<double , 6 ,1> delta_se3;
//     delta_se3.block<3,1>(0,0)=delta_rho;
//     delta_se3.block<3,1>(3,0)=delta_phi;

//     T_plus_delta = Sophus::SE3d::exp(delta_se3) * T;

//     // T_plus_delta = T * Sophus::SE3d::exp(delta);
//     return true;
//   }

//   // Jacobian of SE3 plus operation for Ceres
//   //
//   // Dx T * exp(x)  with  x=0
//   //
//   virtual bool ComputeJacobian(double const* T_raw,
//                                double* jacobian_raw) const {
//     Eigen::Map<Sophus::SE3d const> T(T_raw);
//     Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
//         jacobian_raw);
//     // jacobian = T.Dx_this_mul_exp_x_at_0();
//     jacobian.setZero();
//     jacobian.block<3,3>(4,0) = Eigen::Matrix3d::Identity();
//     jacobian.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
//     return true;
//   }

//   virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }

//   virtual int LocalSize() const { return Sophus::SE3d::DoF; }
// };
