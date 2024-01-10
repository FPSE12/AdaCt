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


struct  ConstantVelocitySEO3{
    ConstantVelocitySEO3(const Sophus::Vector3d &previous_vel, double beta): previous_velocity_(previous_vel), beta_(beta){}

    template<typename T>
    bool operator()(const T * const begin_T, const  T *const end_T, T * residual) const{

    }
private:
    Sophus::Vector3d previous_velocity_;
    double beta_=1.0;
};

struct MultiModeConstantVelocity{
    MultiModeConstantVelocity(const Eigen::Vector3d &previoud_velocity, double beta ):previou_velocity_(previoud_velocity),beta_(beta){}

    template<typename T>
    bool operator()(const T * const begin_t, const T* const mid_t ,const T *const end_t, T * residual) const{
        residual[0]=beta_*(mid_t[0]-begin_t[0]-previou_velocity_(0,0));
        residual[1]=beta_*(mid_t[1]-begin_t[1]-previou_velocity_(1,0));
        residual[2]=beta_*(mid_t[2]-begin_t[2]-previou_velocity_(2,0));

        residual[3]=beta_*(end_t[0]-mid_t[0]-mid_t[0]+begin_t[0]);
        residual[4]=beta_*(end_t[1]-mid_t[1]-mid_t[1]+begin_t[1]);
        residual[5]=beta_*(end_t[2]-mid_t[2]-mid_t[2]+begin_t[2]);


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


//-----------------------THree test------------------------
struct CTFunctor3{

    CTFunctor3(double alpha,const V3D &raw_point,const V3D &ref_points, double weight, V3D normal)
            :alpha_(alpha),raw_points_(raw_point),ref_points_(ref_points),weight_(weight),normal_(normal) {}


    template<class T>//必须使用模板
    bool operator()( const T *const begin_rot,const T *const mid_rot,
                     const T *const end_rot, const T * const begin_trans, const T *const mid_trans ,const T * const end_trans,T * residual)const{

        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_mid(const_cast<T *>(mid_rot));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_begin(const_cast<T *>(begin_trans));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_mid(const_cast<T*>(mid_trans));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_end(const_cast<T *>(end_trans));
        Eigen::Quaternion<T> quad_inter;
        Eigen::Matrix<T, 3, 1> tr;

        if(alpha_<0.5){
            double Talpha_ = alpha_*2;
            quad_inter=quat_begin.normalized().slerp(T(Talpha_),quat_mid.normalized());

            quad_inter.normalize();



            tr(0, 0) = (1-Talpha_) * trans_begin(0,0) + Talpha_ * trans_mid(0,0);
            tr(1, 0) = (1-Talpha_) * trans_begin(1,0) + Talpha_ * trans_mid(1,0);
            tr(2, 0) = (1-Talpha_) * trans_begin(2,0) + Talpha_ * trans_mid(2,0);
        }else{
            double  Talpha_=(alpha_-0.5)*2;
            quad_inter=quat_begin.normalized().slerp(T(Talpha_),quat_end.normalized());

            quad_inter.normalize();



            tr(0, 0) = (1-Talpha_) * trans_begin(0,0) + Talpha_ * trans_end(0,0);
            tr(1, 0) = (1-Talpha_) * trans_begin(1,0) + Talpha_ * trans_end(1,0);
            tr(2, 0) = (1-Talpha_) * trans_begin(2,0) + Talpha_ * trans_end(2,0);
        }



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

//-------------------edge----------------------
struct LidarEdgeFactor {
    LidarEdgeFactor( Eigen::Vector3d curr_point_,  Eigen::Vector3d last_point_a_,
                     Eigen::Vector3d last_point_b_, double s_, double alpha_)
            : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_), alpha(alpha_) {}


    template<typename T>
    bool operator()( const T *const begin_rot,
                     const T *const end_rot, const T * const begin_trans,  const T * const end_trans,T * residual)const{


        //将double数组转为eigen，必须写为模板（因为使用自动求导得到雅各比矩阵）
        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_begin(const_cast<T *>(begin_trans));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_end(const_cast<T *>(end_trans));
        Eigen::Quaternion<T> quad_inter=quat_begin.normalized().slerp(T(alpha),quat_end.normalized());

        quad_inter.normalize();

        Eigen::Matrix<T, 3, 1> tr;

        tr(0, 0) = (1-alpha) * trans_begin(0,0) + alpha * trans_end(0,0);
        tr(1, 0) = (1-alpha) * trans_begin(1,0) + alpha * trans_end(1,0);
        tr(2, 0) = (1-alpha) * trans_begin(2,0) + alpha * trans_end(2,0);

        //将当前帧的点变换到上一帧坐标系中
        Eigen::Matrix<T, 3, 1> lp = quad_inter * curr_point.template cast<T>() + tr;



        //根据平行四边形的面积计算得到距离
        Eigen::Matrix<T, 3, 1> nu = (lp - last_point_a.template cast<T>()).cross(lp - last_point_b.template cast<T>());
        Eigen::Matrix<T, 3, 1> de = last_point_a.template cast<T>() - last_point_b.template cast<T>();


        //距离：num.norm()/de.norm();
        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();

        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, const double s_, const double alpha_)
    {
        //自动求导；LidarEdgeFactor：残差类型；3：残差维度；4：优化变量q维度；3：优化变量t维度
        return (new ceres::AutoDiffCostFunction<
                LidarEdgeFactor, 3, 4,4,3,3>(
                new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_, alpha_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
    double alpha;
};

//struct LidarPlaneFactor
//{
//    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
//                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_, double alpha_)
//            : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
//              last_point_m(last_point_m_), s(s_), alpha(alpha_)
//    {
//        //计算法向量；corss叉乘
//        ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
//        //法向量归一化
//        ljm_norm.normalize();
//    }

    //计算残差
//    template <typename T>
//    bool operator()( const T *const begin_rot,
//                     const T *const end_rot, const T * const begin_trans,  const T * const end_trans,T * residual) const
//    {
//
//        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot));
//        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot));
//        Eigen::Map<Eigen::Matrix<T,3,1>> trans_begin(const_cast<T *>(begin_trans));
//        Eigen::Map<Eigen::Matrix<T,3,1>> trans_end(const_cast<T *>(end_trans));
//        Eigen::Quaternion<T> quad_inter=quat_begin.normalized().slerp(T(alpha),quat_end.normalized());
//
//        quad_inter.normalize();
//
//        Eigen::Matrix<T, 3, 1> tr;
//
//        tr(0, 0) = (1-alpha) * trans_begin(0,0) + alpha * trans_end(0,0);
//        tr(1, 0) = (1-alpha) * trans_begin(1,0) + alpha * trans_end(1,0);
//        tr(2, 0) = (1-alpha) * trans_begin(2,0) + alpha * trans_end(2,0);
//
//
//        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
//        Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
//        //Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
//        //Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
//        Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};
//
//        //当前帧的点变换到上一帧的坐标系中
//        Eigen::Matrix<T, 3, 1> lp;
//        lp = quad_inter * cp + tr;
//
//
//        //内积：|lp-lpj|*|ljm|*cos； |ljm|=1
//        //构造残差
//        residual[0] = (lp - lpj).dot(ljm);
//
//        return true;
//    }
//
//    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
//                                       const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
//                                       const double s_, const double alpha_)
//    {
//        //AuoDiffCostFunction：自动求导；LidaPlaneFACTORE
//        return (new ceres::AutoDiffCostFunction<
//                LidarPlaneFactor, 1, 4,4,3, 3>(
//                new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_, alpha_)));
//    }
//
//    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
//    //j.l.m平面的法向量
//    Eigen::Vector3d ljm_norm;
//    double s;
//    double alpha;
//};


struct LidarPlaneNormFactor
{

    LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
                         double negative_OA_dot_norm_,double alpha_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
                                                         negative_OA_dot_norm(negative_OA_dot_norm_),alpha(alpha_) {}

    template <typename T>
    bool operator()( const T *const begin_rot,
                     const T *const end_rot, const T * const begin_trans,  const T * const end_trans,T * residual) const
    {
        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_begin(const_cast<T *>(begin_trans));
        Eigen::Map<Eigen::Matrix<T,3,1>> trans_end(const_cast<T *>(end_trans));
        Eigen::Quaternion<T> quad_inter=quat_begin.normalized().slerp(T(alpha),quat_end.normalized());

        quad_inter.normalize();

        Eigen::Matrix<T, 3, 1> tr;

        tr(0, 0) = (1-alpha) * trans_begin(0,0) + alpha * trans_end(0,0);
        tr(1, 0) = (1-alpha) * trans_begin(1,0) + alpha * trans_end(1,0);
        tr(2, 0) = (1-alpha) * trans_begin(2,0) + alpha * trans_end(2,0);
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = quad_inter * cp + tr;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
        //点到平面的距离，norm已经归一化了
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_, const double alpha_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPlaneNormFactor, 1,4,4,3,3>(
                new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_,alpha_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    double alpha;
};



