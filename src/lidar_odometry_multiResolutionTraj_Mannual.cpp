#include "AdaCt/utility.h"
#include "AdaCt/frame_info_usecommonPointTypeOnly.hpp"
#include "AdaCt/cloud_convert.hpp"
#include "AdaCt/downSample.hpp"
#include "AdaCt/trajectory.hpp"
//#include <ikd-Tree/ikd_Tree.h>
#include "AdaCt/costfunction.hpp"

#include "voxelmap/voxelmap_OctoTree.hpp"
//
//kdMap have to put before main()

//KD_TREE<pcl::PointXYZ> ikd_tree;

//#define USE_MANNUAL 1

#define MAX_ITER_COUNT 5
#define MAX_ITER_COUNT_SCAN 2

#define MAX_NEIGHBOR_NUM 20

#define A2D_THRESHOLD 0.0 //UNSTRUCTE:low; , structure: >0.4

#define keyframe_threshold_w 5
#define keyframe_threshold_t 0.3

//#define keyframe_valid true
//



class Lidar_odo : public configParam
{
public:
    ros::Subscriber lidar_sub;
    ros::Subscriber lidar_sub_livox;

    ros::Publisher map_pub;
    ros::Publisher odo_pub;
    ros::Publisher traj_pub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_ori_pub;
    ros::Publisher cloud_valid_pub;
    ros::Publisher cloud_world_pub;
    ros::Publisher cloud_downsample_pub;


    // cloud
    std::deque<sensor_msgs::PointCloud2> rosCloudQue;
    std::deque<livox_ros_driver::CustomMsg> rosLivoxCloudQue;

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_valid;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_world;
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_deskew;
    //global map
    pcl::PointCloud<PointXYZIRT>::Ptr globalMap;

    //global traj
    nav_msgs::Path globalPath;

   // double timeStart, timeEnd, headertime;
//    frame_info curr_frame;

    std::mutex mtx;
    std::mutex mtx_get;

    bool first_flag;
    int frame_count;

    //motion_threshold
    bool isMove = false;
    Sophus::SE3d model_deviation;
    double min_motion_th_ = 0.1;
    double initial_threshold_ = 1.5;
    int num_samples_ = 0;
    double modle_error_sse2 = 0.0;
    double reg_thres  = 1.5;

    // map
    tsl::robin_map<Voxel, OctoTree*> voxel_map;
    float max_voxel_size;
    int max_layer;
    vector<int> layer_size;
    int max_points_size;
    float min_eigen_value;

    //PCA analyse
    Eigen::Vector3f PCALastMainDir;
    Eigen::Vector3f PCALastEigenValue;

    //env judge
    double average_dis = 0;


    //trajectory
    std::vector<OptPose> key_poses;
    OptPose last_pose;
    std::vector<PointType> last_points;

    double trajectory_startTime;
    Trajectory globalTraj;


    int iter_nums; // 5

    //time cost ms
    double init_cost;
    double neighbor_find_average;
    double solve_average;
    double solve_cost;
    double map_cost;
    double all_cost;
    double cost_threshold;
    double cost_robust;

    double a2d_average;


    Lidar_odo()
    {
        //ROS_INFO("1");
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Lidar_odo::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        //lidar_sub_livox = nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 5, &Lidar_odo::livoxLidarCallback, this, ros::TransportHints().tcpNoDelay());
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/global_map",1);
        odo_pub = nh.advertise<nav_msgs::Odometry>("adact/odometry",1);
        traj_pub = nh.advertise<nav_msgs::Path>("adact/path",1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/deskew_pointcloud",1);
        cloud_ori_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/ori_pointcloud",1);
        cloud_valid_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/valid_points",1);
        cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/world_points",1);
        cloud_downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("adact/downsample",1);


        cloud_valid.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_world.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_deskew.reset(new pcl::PointCloud<PointXYZIRT>());
        cloud_ori.reset(new pcl::PointCloud<pcl::PointXYZ>());
        globalMap.reset(new pcl::PointCloud<PointXYZIRT>());

       // Segcloud.reset(new pcl::PointCloud<PointXYZIRT>());

        rosCloudQue.clear();

        //time cost
        init_cost = 0;
        neighbor_find_average = 0;
        solve_average = 0;
        solve_cost = 0;
        map_cost = 0;
        all_cost = 0;
        cost_threshold = 100.0;
        cost_robust = 30.0;


        // params init
        first_flag = true;
        frame_count =-1;
        iter_nums =MAX_ITER_COUNT;

        isMove = false;


        last_pose.initialMotion();
        //mapping params
        max_voxel_size = 0.8;
        max_layer = 2;//0,1,2
        layer_size=vector<int>{10,5,5,5,5};
        max_points_size = 1000;//1000
        min_eigen_value = 0.01;


        last_points.clear();



    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        rosCloudQue.push_back(*laserMsg);
        //preprocess();
    }

    void livoxLidarCallback(const livox_ros_driver::CustomMsgConstPtr &laserMsg){
        std::lock_guard<std::mutex> lock(mtx);
        rosLivoxCloudQue.emplace_back(*laserMsg);
    }





    //point2point
    //source_cloud is all last_frame in world axis
    void SCAN2SCAN(vector<PointType>& targetCloud, vector<PointType > &sourceCloud, OptPose &curr_pose_){

//        std::cout<<curr_pose_.end_pose.translation()<<std::endl;
        //time cost??
        tsl::robin_map<Voxel,vector<PointType>> grid;
        double downsample_size = 0.2;
        Voxel voxel;
        for(auto  s_point : sourceCloud ){
            voxel.x = static_cast<short>(s_point.point_world[0] / downsample_size);
            if(voxel.x<0) voxel.x--;
            voxel.y = static_cast<short>(s_point.point_world[1] / downsample_size);
            if(voxel.y<0) voxel.y--;
            voxel.z = static_cast<short>(s_point.point_world[2] / downsample_size);
            if(voxel.z<0) voxel.z--;
            grid[voxel].push_back(s_point);
        }
        for(int iter =0; iter<5;iter++){
            ceres::Problem problem;
            ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
            Eigen::Quaterniond  begin_quat_ = curr_pose_.beginQuat();
            Eigen::Vector3d  begin_trans_ = curr_pose_.beginTrans();
            Eigen::Quaterniond end_quat_ = curr_pose_.endQuat();
            Eigen::Vector3d end_trans_ = curr_pose_.endTrans();


            problem.AddParameterBlock(begin_quat_.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(end_quat_.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(begin_trans_.data(), 3);
            problem.AddParameterBlock(end_trans_.data(), 3);
            int valid_num=0;
            for(auto  t_point : targetCloud){

                t_point.point_world = curr_pose_.linearInplote(t_point.alpha)* t_point.point;

                voxel.x = static_cast<short>(t_point.point_world[0] / downsample_size);
                if(voxel.x<0) voxel.x--;
                voxel.y = static_cast<short>(t_point.point_world[1] / downsample_size);
                if(voxel.y<0) voxel.y--;
                voxel.z = static_cast<short>(t_point.point_world[2] / downsample_size);
                if(voxel.z<0) voxel.z--;

                if(grid.find(voxel)!=grid.end()){
                    double min_dis = std::numeric_limits<double>::max();
                    PointType target_point;
                    for(auto & s_point : grid[voxel]){
                        double dis = sqrt((s_point.point_world[0] - t_point.point_world[0])*(s_point.point_world[0] - t_point.point_world[0])
                                          +(s_point.point_world[1] - t_point.point_world[1])*(s_point.point_world[1] - t_point.point_world[1])
                                          +(s_point.point_world[2] - t_point.point_world[2])*(s_point.point_world[2] - t_point.point_world[2]));

                        if(dis<min_dis){
                            min_dis = dis;
                            target_point = s_point;
                        }
                    }

                    valid_num++;
                    double alpha = t_point.alpha;
                    double weight = exp(-min_dis);
                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<Point2pointFunctor, 1, 4, 4, 3, 3>(
                            new Point2pointFunctor(alpha, weight, target_point.point_world, t_point.point));

                    problem.AddResidualBlock(cost_function,
                                             loss_function,
                                             begin_quat_.coeffs().data(),end_quat_.coeffs().data(), begin_trans_.data(), end_trans_.data());
                }

            }

//            ROS_INFO("valid: %d",valid_num);
            ceres::Solver::Options option;
            option.linear_solver_type = ceres::DENSE_QR;
            option.max_num_iterations =6;
            option.num_threads = 16;
            option.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
            ceres::Solver::Summary summary;
            ceres::Solve(option, &problem, &summary);

            begin_quat_.normalize();
            end_quat_.normalize();
            curr_pose_.begin_pose=Sophus::SE3d(begin_quat_, begin_trans_);
            curr_pose_.end_pose=Sophus::SE3d(end_quat_, end_trans_);


        }
//        std::cout<<curr_pose_.end_pose.translation()<<std::endl;

    }

    void getSegCloud(std::vector<PointType>& cloud_ori,
                     double timeStart, double timeEnd,
                     double timeSegBegin, double timeSegEnd,
                     std::vector<PointType>& SegCloud){
        SegCloud.clear();
        if(timeSegBegin < timeStart || timeSegEnd > timeEnd || timeSegBegin > timeSegEnd){
            ROS_ERROR("cloud Seg false! timeSegBegin: %f, timeSegEnd: %f, timeStart: %f, timeEnd:%f ", timeSegBegin,
                      timeSegEnd,timeStart,timeEnd);
            ROS_BREAK();
        }
        for(PointType &point : cloud_ori){
            if(point.timestamp>= timeSegBegin && point.timestamp <= timeSegEnd){
                SegCloud.emplace_back(point);
            }
        }
    }

    inline double computerModelError(const Sophus::SE3d & model_deviation, double max_range){
        const double theta =
                Eigen::AngleAxisd(model_deviation.rotationMatrix()).angle();
        const double delta_theta = 2.0 * max_range * std::sin(theta/ 2.0);
        const double delta_trans = model_deviation.translation().norm();
        return delta_trans + delta_theta;
    }

    inline void computeThreshold(){
        double model_error = computerModelError(model_deviation,100);
        //ROS_INFO("MODEL_ERROR：%f", model_error);

        if(model_error > min_motion_th_){
            modle_error_sse2 += model_error * model_error;
            num_samples_++;
        }
        if(num_samples_<1){
            reg_thres = initial_threshold_;
        }
        reg_thres = std::sqrt(modle_error_sse2 / num_samples_);
        //ROS_INFO("REG_THRES: %f", reg_thres);
    }

    void ENVAnalyse(vector<PointType> & points_vec){
        auto evaluate = std::chrono::steady_clock::now();
        int sample =0;
        average_dis = 0;
        int count=0;
        for(auto & point :points_vec){
            average_dis += point.distance;
            sample++;
        }
        auto  evaluatea_end = std::chrono::steady_clock::now();
        average_dis = average_dis/sample;
        ROS_INFO("DISTANCE: %f", average_dis);//<5 0.2  <10 0.4 >10 ada

        if(average_dis<5.0){
            envtype = ENVTYPE::NARROW;
            keyframe_valid = true;
            seg_frame = false;
            ROS_WARN("NARROW");
        }else if(average_dis < 8.0){
            envtype = ENVTYPE::INDOOR;
            keyframe_valid = false;
            seg_frame = false;
        } else if(average_dis <15.0){
            envtype = ENVTYPE::OUTDOOR;
            keyframe_valid = false;
            seg_frame = true;
        } else{
            envtype = ENVTYPE::OPEN;
            keyframe_valid = false;
            seg_frame = true;
        }
    }

    int PCAAnalyse(){
        int seg_num=1;
        Eigen::Vector3f PCACurrMainDir;
        Eigen::Vector4f pcaCenter;
        Eigen::Vector3f PCACurrEigenValue;
        pcl::compute3DCentroid(*cloud_ori, pcaCenter);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud_ori, pcaCenter, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver_(covariance,Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorPCA = eigen_solver_.eigenvectors();
        PCACurrMainDir = eigenVectorPCA.col(2);
        PCACurrEigenValue = eigen_solver_.eigenvalues();
        if(!first_flag){
            //angle change
            double value = abs((PCACurrEigenValue[2] / PCALastEigenValue[2])-1.0);
            double angle = acos(PCACurrMainDir.dot(PCALastMainDir)/(PCACurrMainDir.norm()*PCALastMainDir.norm()));
            angle = angle/M_PI * 180;
            if(angle>90.0){
                angle = 180.0-angle;
            }
            ROS_INFO("angle %f && value: %f",angle,value);

            if(value >= 0.15){
                keyframe_valid = true;
                envfeature = ENVFEATURE::DYNAMIC;
                seg_frame = false;
                ROS_WARN("DYNAMIC");
            }else{
                keyframe_valid = false;
                envfeature = ENVFEATURE::STATIC;
                if(angle>=20.0){
//                    seg_num=4;
//                    if(envtype <= ENVTYPE::INDOOR){
//                        seg_num = 2;
//                    }
//                    ego_motion_grade = MOTION_GRADE::EXTREME;
//                    ROS_WARN("MOTION ARRESIVE");
//                }else if(angle >=20.0){

                    //seg_frame =true;
                    seg_num =3;
                    if(envtype <= ENVTYPE::INDOOR){
                        seg_num = 2;
                    }
                    ego_motion_grade = MOTION_GRADE::AGGRESSIVE;
                }else if(angle>=2.0){
                    ego_motion_grade = MOTION_GRADE::TURN;
                    seg_num = 2;
                }else{
                    ego_motion_grade = MOTION_GRADE::SMOOTH;
                    seg_num=1;
                }

            }



        }
        PCALastMainDir = PCACurrMainDir;
        PCALastEigenValue = PCACurrEigenValue;
        return seg_num;
    }

    void Initframe(frame_info & curr_frame,
                   std::vector<PointType> & points_vec,
                   Sophus::SE3d begin_pose, Sophus::SE3d end_pose,
                   double timeB, double timeE, double header_time,
                   int frame_id)
    {
        curr_frame.Reset();
        curr_frame.pose.begin_pose = begin_pose;
        curr_frame.pose.end_pose  = end_pose;


        curr_frame.timeStart = timeB;
        curr_frame.timeEnd = timeE;
        curr_frame.headertime = header_time;
        curr_frame.frame_id = frame_id;
        curr_frame.setPoints(points_vec);

        //下采样会导致问题

        max_iter_nums = 5;
        switch (envtype) {
            case ENVTYPE::NARROW:
                if(ego_motion_grade >= MOTION_GRADE::AGGRESSIVE){
                    curr_frame.grid_sample_mid_in_vec(0.1);
                    //max_iter_nums = 10;
                }else{
                    //curr_frame.Adaptive_sample_mid_in_vec_NARROW(average_dis);
                    curr_frame.grid_sample_mid_in_vec(0.2);
                }

                break;
            case ENVTYPE::INDOOR:

                //curr_frame.grid_sample_mid_in_vec(0.2);

                curr_frame.Adaptive_sample_mid_in_vec_INDOOR(average_dis);
                break;
            case ENVTYPE::OUTDOOR:
                curr_frame.Adaptive_sample_mid_in_vec(average_dis);
                break;
            case ENVTYPE::OPEN:
                curr_frame.Adaptive_sample_mid_in_vec_OPEN(average_dis);
                break;
            default:
                curr_frame.Adaptive_sample_mid_in_vec_OPEN(average_dis);
                break;

        }

//          curr_frame.grid_sample_mid_in_vec(0.1);
//        curr_frame.Adaptive_sample_mid_in_vec();

        //if seg the cloud scan2scan: use the last full cloud
//        auto scan2scan_start = std::chrono::steady_clock::now();
//        Sophus::SE3d pre_end = curr_frame.pose.end_pose;
//        SCAN2SCAN(curr_frame.points, last_points,curr_frame.pose);
//        Sophus::SE3d aft_end = curr_frame.pose.end_pose;
//        Sophus::SE3d correct = aft_end.inverse()*pre_end;
//        auto scan2scan_end = std::chrono::steady_clock::now();
//        ROS_INFO("SCAN2SCAN SPEND:%f",std::chrono::duration<double,std::milli>(scan2scan_end - scan2scan_start).count());
//        ROS_INFO("CORRECT: %f, %f,%f,%f,%f,%f",correct.angleX(),correct.angleY(),correct.angleZ(),correct.translation().x(),
//                 correct.translation().y(),correct.translation().z());
    }

    void Solve(frame_info & curr_frame, int & motion_evaluate, int seg_num){
        const double th = reg_thres / 3;
        const double dis_thres = reg_thres * 3;
        auto square = [](double x) {return x*x;};
        auto Weight = [&](double residual){
            return square(th) / (square(th) + residual);
        };

        int iter_count = 0;
        double find_neighborcost=0;
        double solve_cost = 0;
        auto iter_start = std::chrono::steady_clock::now();
        for (; iter_count < max_iter_nums; iter_count++) {

            // ceres opt
            ceres::LossFunction *loss_function;
            if(ego_motion_grade == MOTION_GRADE::AGGRESSIVE || ego_motion_grade == MOTION_GRADE::EXTREME){
                loss_function = new ceres::CauchyLoss(0.1 * ego_motion_grade);
            }else{
                loss_function = new ceres::CauchyLoss(0.1);
            }
            //ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
            //ceres::LossFunction * loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem problem;

            Eigen::Quaterniond  begin_quat = curr_frame.beginQuat();
            Eigen::Vector3d  begin_trans = curr_frame.getBeginTrans();
            Eigen::Quaterniond end_quat = curr_frame.endQuat();
            Eigen::Vector3d end_trans = curr_frame.getEndTrans();



            //mannual calculate
#ifdef USE_MANNUAL
            ceres::LocalParameterization *parameterization = new RotationParameterization();
            problem.AddParameterBlock(&begin_quat.x(),4, parameterization);
            problem.AddParameterBlock(&end_quat.x(),4, parameterization);
            problem.AddParameterBlock(&begin_trans.x(), 3);
            problem.AddParameterBlock(&end_trans.x(), 3);
#else
            //auto calculate
            problem.AddParameterBlock(begin_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(end_quat.coeffs().data(),4, new ceres::EigenQuaternionManifold);
            problem.AddParameterBlock(begin_trans.data(), 3);
            problem.AddParameterBlock(end_trans.data(), 3);
#endif

            int opt_num=0;
            auto findNeighbor_start = std::chrono::steady_clock::now();
            auto time1 = std::chrono::steady_clock::now();
            //bool IsValid=local_map.NeighborSearch(curr_frame.cloud_world->points[i],raw_point,searchDis,neighbor,pabcd);
            vector<ptpl> ptpl_list;
            vector<Eigen::Vector3d > no_match_list;
            cloud_valid->clear();
            for(auto & p: curr_frame.points){
                p.point_world = curr_frame.pose.linearInplote(p.alpha) * p.point;
            }

            //ptpl->point is point-body
            BuildResidualListOMP(voxel_map,max_voxel_size,3.0,max_layer, curr_frame.points,ptpl_list,1000,no_match_list);
            auto time2 = std::chrono::steady_clock::now();

            opt_num = ptpl_list.size();
//                if(Debug_print){
//                    ROS_INFO("all_point:%d, valid_num: %d",curr_frame.points.size(), opt_num);
//                }

            for(auto  &pl : ptpl_list){

                double alpha  = pl.point_alpha;
                double weight = pl.A2D;
                if(pl.A2D < A2D_THRESHOLD){
                    continue;
                }

                if(pl.normal.dot(begin_trans - pl.point)<0){
                    pl.normal = -1.0 * pl.normal;
                }

                //double weight = Weight(pl.residual);//not good
//
#ifdef USE_MANNUAL
                CTFunctor2_mannual *cost_function = new CTFunctor2_mannual(alpha, pl.point, pl.center, weight,pl.normal);
                problem.AddResidualBlock(cost_function,
                                         loss_function,
                                         &begin_quat.x(),&end_quat.x(), &begin_trans.x(), &end_trans.x());
#else
                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CTFunctor2, 1, 4, 4, 3, 3>(
                        new CTFunctor2(alpha, pl.point, pl.center, weight,pl.normal));

                problem.AddResidualBlock(cost_function,
                                         loss_function,
                                         begin_quat.coeffs().data(),end_quat.coeffs().data(), begin_trans.data(), end_trans.data());
#endif

                PointXYZIRT p;
                p.x = pl.point[0];
                p.y = pl.point[1];
                p.z = pl.point[2];
                cloud_valid->push_back(p);
            }



            publishCloud(cloud_valid_pub,cloud_valid,curr_frame.headertime,"odometry");//1ms
//
//            publishCloud(cloud_downsample_pub,curr_frame.cloud_ori_downsample,ros::Time(curr_frame.headertime),"odometry");
            auto findNeighbor_end = std::chrono::steady_clock::now();

            find_neighborcost += std::chrono::duration<double, std::milli >(findNeighbor_end-findNeighbor_start).count();

            //add other constraints
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistency,6,4,3>(
                                             new LocationConsistency(last_pose.endQuat(), last_pose.endTrans(), std::sqrt(opt_num * 0.01 *seg_num))),
                                     nullptr,
                                     begin_quat.coeffs().data(),begin_trans.data()

            );


            Eigen::Quaterniond pre_quat_delta = last_pose.beginQuat().conjugate() * last_pose.endQuat();
            pre_quat_delta.normalize();
            Eigen::Vector3d pre_trans_delta = last_pose.endTrans() - last_pose.beginTrans();//in world axis
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocityRotTran,6,4,4,3,3>(
                                             new ConstantVelocityRotTran(pre_quat_delta,pre_trans_delta,std::sqrt(opt_num * 0.01 * seg_num))),
                                     nullptr,
                                     begin_quat.coeffs().data(),end_quat.coeffs().data(),begin_trans.data(),end_trans.data()
            );


            // solve
            ceres::Solver::Options option;
            option.linear_solver_type = ceres::DENSE_QR;
            option.max_num_iterations =8;
            option.num_threads = 8;
            option.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

            ceres::Solver::Summary summary;
            auto solve_start = std::chrono::steady_clock::now();
            ceres::Solve(option, &problem, &summary);

            auto solve_end = std::chrono::steady_clock::now();

            solve_cost += std::chrono::duration<double, std::milli >(solve_end-solve_start).count();

            begin_quat.normalize();
            end_quat.normalize();
            curr_frame.setMotion(begin_quat, end_quat, begin_trans, end_trans);

        }
        auto iter_end = std::chrono::steady_clock::now();

//        if (Debug_print) {
//                    ROS_INFO("neighbor COST: %f ms",
//                             find_neighborcost);
//                    ROS_INFO("compute COST: %f ms",
//                             solve_cost);
////                    ROS_INFO("problem make COST: %f ms",
////                             std::chrono::duration<double, std::milli>(findNeighbor_end - findNeighbor_start).count());
////                    ROS_INFO("OPT_NUM:%d",opt_num);
//        }

        //judge keyframe
        //last_pose = curr_frame.pose;
        if(motion_evaluate == 2){
           return;
        }
        motion_evaluate = curr_frame.pose.compareDiff(key_poses.back(), keyframe_threshold_w, keyframe_threshold_t);



    }

    void ProcessSeg( std::vector<PointType> & points_vec, double timeStart, double timeEnd,
                    double  timeSegBegin, double timeSegEnd, double headertime,
                    int frame_id, int & motion_evaluate,
                    std::vector<PointType> & points_vec_process, int seg_num){

        auto procee_begin = std::chrono::steady_clock::now();

        auto seg_begin = std::chrono::steady_clock::now();
        std::vector<PointType> SegPoints;
        getSegCloud(points_vec,timeStart, timeEnd,
                    timeSegBegin, timeSegEnd,SegPoints);
        auto seg_end =std::chrono::steady_clock::now();


        auto init_begin= std::chrono::steady_clock::now();
        Sophus::SE3d begin_pose = globalTraj.predict(timeSegBegin);
        Sophus::SE3d end_pose = globalTraj.predict(timeSegEnd);
        Sophus::SE3d T_pred = end_pose;
//        if(!isMove){
//            isMove  = T_pred.translation().norm()>0.5;
//        }else{
//            computeThreshold();
//        }
        frame_info curr_frame;
        //will clear Segpoints
        Initframe(curr_frame, SegPoints, begin_pose,end_pose, timeSegBegin, timeSegEnd, headertime, frame_id);
        auto init_end = std::chrono::steady_clock::now();

        auto solve_begin = std::chrono::steady_clock::now();
        Solve(curr_frame, motion_evaluate,seg_num);
        auto solve_end = std::chrono::steady_clock::now();

        auto update_begin = std::chrono::steady_clock::now();
        curr_frame.getWorldPoints(points_vec_process);
        //globalTraj.pop_back();
        globalTraj.addPose(curr_frame.pose.begin_pose,curr_frame.timeStart);
        globalTraj.addPose(curr_frame.pose.end_pose,curr_frame.timeEnd);

        last_pose = curr_frame.pose;
        model_deviation = T_pred.inverse() * curr_frame.pose.end_pose;
        auto update_end = std::chrono::steady_clock::now();
        auto process_end = std::chrono::steady_clock::now();

//        if(Debug_print){
//            ROS_INFO("PROCESS SEG COST:------");
//            double  process_cost = std::chrono::duration<double, std::milli > (process_end - process_end).count();
//            double  segcost = std::chrono::duration<double, std::milli > (seg_end - seg_begin).count();
//            double  initcost = std::chrono::duration<double, std::milli > (init_end - init_begin).count();
//            double  solvecost = std::chrono::duration<double, std::milli > (solve_end - solve_begin).count();
//            double  updatecost = std::chrono::duration<double, std::milli > (update_end - update_begin).count();
//            ROS_INFO("peocess cost all : %f ms \n seg_cost: %f ms \n init_cost: %f ms \n  solve_cost: %f ms \n update_cost: %f ms",
//                     process_cost, segcost, initcost, solvecost, updatecost);
//
//        }
    }

    void updateworldPointFromGlobalTrajectory(std::vector<PointType> & points_vec){
        for(auto &point_type : points_vec){
            SE3 point_pose = globalTraj.getPose(point_type.timestamp);
            point_type.point_world = point_pose * point_type.point;
            point_type.point_deskew = globalTraj.getLastPose().inverse() * point_type.point_world;
        }
    }

    void AddCloudToMap(int motion_evaluate_, std::vector<PointType> &points_vec_process, double & header_stamp){
        auto map_start = std::chrono::steady_clock::now();
//            curr_frame.updateWorldCloud();
        //updateworldPointFromGlobalTrajectory(points_vec);
        transVec2Pcl(points_vec_process);
        if( keyframe_valid || motion_evaluate_==2 ){
//                curr_frame.updateFromDownSample();
//                curr_frame.updateWorldCloud();

            last_points.clear();
            last_points.resize(points_vec_process.size());
            copy(points_vec_process.begin(),points_vec_process.end(),last_points.begin());

            updateVoxelMap(points_vec_process, max_voxel_size, max_layer, layer_size,
                           max_points_size, max_points_size, min_eigen_value,
                           voxel_map);

            key_poses.push_back(last_pose);

            if(map_pub.getNumSubscribers()){

                globalMap->clear();
                getVoxelMap(voxel_map, globalMap);

//            pub map
//                *globalMap += *cloud_world;
                publishCloud(map_pub,globalMap,header_stamp,"map");
            }


        }
        auto map_end = std::chrono::steady_clock::now();
        map_cost = std::chrono::duration<double,std::milli>(map_end-map_start).count();
        ROS_INFO("MAP UPDATE: %f ms", map_cost);
    }



    void Process()
    {
        ros::Rate r(100);//100Hz

        while (ros::ok())
        {

            if (rosCloudQue.empty() && rosLivoxCloudQue.empty())
            {
                continue;
                // return;
            }

            auto all_start = std::chrono::steady_clock::now();

            frame_count++;
            ROS_INFO("------------------------------frame_id:%d----------------------------",frame_count);

            //pcl::PointCloud<PointXYZIRT>::Ptr cloud_ori(new pcl::PointCloud<PointXYZIRT>());
            cloud_ori->clear();

            std::vector<PointType> points_vec;
            std::vector<PointType> points_vec_process;


            // get timestamp
            double headertime = 0.0;
            double timeStart = 0.0;
            double timeEnd = 0.0;

            //get cloud
            mtx_get.lock();
            if((LidarType)lidar_type == AVIA){
                //ROS_INFO("1");
                livox_ros_driver::CustomMsg cloud_ori_ros = std::move(rosLivoxCloudQue.front());
                rosLivoxCloudQue.pop_front();
                CLOUD_CONVERT cc(cloud_ori_ros,points_vec,headertime,timeStart, timeEnd);
            }else{
                sensor_msgs::PointCloud2 cloud_ori_ros = std::move(rosCloudQue.front());
                cloud_ori->clear();
                pcl::fromROSMsg(cloud_ori_ros,*cloud_ori);
                rosCloudQue.pop_front();
                CLOUD_CONVERT cc(cloud_ori_ros,(LidarType)lidar_type, points_vec,headertime,timeStart,timeEnd);
            }
            mtx_get.unlock();
//            pcl::moveFromROSMsg(cloud_ori_ros, *cloud_ori);
//            preprocess(cloud_ori, points_vec,headertime,timeStart, timeEnd);


            if (first_flag )
            {
                // do some initial work
                //ROS_INFO("1");

                //evaluate dir
                ENVAnalyse(points_vec);
                PCAAnalyse();
                frame_info curr_frame;
                curr_frame.setFrameTime(headertime,timeStart, timeEnd,frame_count);
                curr_frame.pose.initialMotion();

                for(auto &point : points_vec){
                    point.point_world = point.point;
                    point.point_deskew = point.point;
                }

                //build map
                buildVoxelMap(points_vec, max_voxel_size, max_layer, layer_size,
                              max_points_size, max_points_size, min_eigen_value,
                              voxel_map);

                last_pose=curr_frame.pose;
                key_poses.push_back(curr_frame.pose);

                trajectory_startTime = timeStart;
                globalTraj.addPose(KNOT::Indentity(timeStart, true));
                globalTraj.addPose(KNOT::Indentity(timeEnd));

                //ROS_INFO("2");
                first_flag = false;
                publish(points_vec, headertime);
                std::vector<PointType>().swap(points_vec);
                //publishCloud(map_pub,globalMap,headertime,"map");
                //ROS_INFO("3");
                continue;
                //return;
            }

            keyframe_valid = false;
            ENVAnalyse(points_vec);
            int motion_evaluate =0;

            int seg_num = PCAAnalyse();
            seg_num = seg_frame?seg_num:1;

            //int seg_num = 2;
            double seg_length = (timeEnd - timeStart)/((double)seg_num);
            double seg_start = timeStart;
            double seg_end  = timeStart;
            for(int seg_index=0;seg_index<seg_num;seg_index++){
               seg_start = seg_end;
               seg_end = min(seg_start + seg_length, timeEnd);
               ProcessSeg(points_vec,timeStart, timeEnd, seg_start, seg_end, headertime,
                          frame_count, motion_evaluate, points_vec_process,seg_num);
            }


            AddCloudToMap(motion_evaluate, points_vec_process, headertime);

            //publish ori_cloud, world_cloud
            publish(points_vec_process,headertime);


            std::vector<PointType>().swap(points_vec);
            std::vector<PointType>().swap(points_vec_process);


            auto all_end = std::chrono::steady_clock::now();
            all_cost = std::chrono::duration<double, std::milli>(all_end-all_start).count();

            if(Debug_print){
                ROS_INFO("ALL COST: %f ms.", all_cost);
                if(all_cost>cost_threshold){
                    ROS_WARN("COST TOO MUCH!");
                }
            }

            //release


            r.sleep();
        }
    }


    void publishCloud(ros::Publisher &pub, pcl::PointCloud<PointXYZIRT>::ConstPtr pcl_cloud, double &header_stamp ,string frame){
        if(pub.getNumSubscribers()) {
            ros::Time ros_headertime = ros::Time(header_stamp);
            sensor_msgs::PointCloud2 ros_cloud;
            pcl::toROSMsg(*pcl_cloud, ros_cloud);
            ros_cloud.header.stamp = ros_headertime;
            ros_cloud.header.frame_id = frame;
            pub.publish(ros_cloud);
            return;
        }
    }

    void transVec2Pcl(std::vector<PointType> &points_vec){
        cloud_world->clear();
        cloud_deskew->clear();
        for(auto & point_type : points_vec){
            Eigen::Vector3d world_point = point_type.point_world;
            Eigen::Vector3d deskew_point = point_type.point_deskew;
            Eigen::Vector3d point = point_type.point;
            PointXYZIRT p;
//            p.x = point[0];
//            p.y = point[1];
//            p.z = point[2];
//            p.intensity = point_type.intensity;
//            cloud_ori->points.emplace_back(p);
            p.x = world_point[0];
            p.y = world_point[1];
            p.z = world_point[2];
            cloud_world->points.emplace_back(p);
            p.x = deskew_point[0];
            p.y = deskew_point[1];
            p.z = deskew_point[2];
            cloud_deskew->points.emplace_back(p);
        }


    }

    void publish(std::vector<PointType> & points_vec, double headertime) {
        nav_msgs::Odometry odometry_pose;
        odometry_pose.header.stamp = ros::Time(headertime);
        odometry_pose.header.frame_id = odometry_frame;
        odometry_pose.pose.pose.orientation.w = last_pose.endQuat().w();
        odometry_pose.pose.pose.orientation.x = last_pose.endQuat().x();
        odometry_pose.pose.pose.orientation.y = last_pose.endQuat().y();
        odometry_pose.pose.pose.orientation.z = last_pose.endQuat().z();

        odometry_pose.pose.pose.position.x = last_pose.endTrans().x();
        odometry_pose.pose.pose.position.y = last_pose.endTrans().y();
        odometry_pose.pose.pose.position.z = last_pose.endTrans().z();
        odo_pub.publish(odometry_pose);


        geometry_msgs::PoseStamped geo_odometry_pose;
        geo_odometry_pose.header = odometry_pose.header;
        geo_odometry_pose.pose = odometry_pose.pose.pose;

        globalPath.header.stamp = ros::Time(headertime);
        globalPath.header.frame_id = "map";
        globalPath.poses.emplace_back(geo_odometry_pose);
        traj_pub.publish(globalPath);

        if(cloud_pub.getNumSubscribers()){
            sensor_msgs::PointCloud2 deskew_cloud;
            pcl::toROSMsg(*cloud_deskew,deskew_cloud);
            deskew_cloud.header.stamp = ros::Time(headertime);
            deskew_cloud.header.frame_id= "odometry";
            cloud_pub.publish(deskew_cloud);
        }

        if(cloud_world_pub.getNumSubscribers()){
            sensor_msgs::PointCloud2 world_cloud;
            pcl::toROSMsg(*cloud_world,world_cloud);
            world_cloud.header.stamp = ros::Time(headertime);
            world_cloud.header.frame_id="map";
            cloud_world_pub.publish(world_cloud);
        }

        if(cloud_ori_pub.getNumSubscribers()){
            sensor_msgs::PointCloud2 ori_cloud;
            pcl::toROSMsg(*cloud_ori,ori_cloud);
            ori_cloud.header.stamp = ros::Time(headertime);
            ori_cloud.header.frame_id= "odometry";
            cloud_ori_pub.publish(ori_cloud);
        }












    }


};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "adact_odometry");
    ROS_INFO("START LIDAR PREPROCESS!");

//
    Lidar_odo lp;
//
//    // // 由于process是循环，如果直接运行，不会进入下面的spin，就不会进入点云回调函数，所以要新开一个线程
    std::thread process(&Lidar_odo::Process, &lp);
//
    //process.join();
//    // // spin才会进入回调函数
    ros::spin();
//
//    // // 加入线程
    process.join();
    Eigen::Quaterniond test;
    test.vec();
    return 0;
}