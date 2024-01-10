//
// Created by wjj on 2024/1/6.
//

#ifndef SRC_CLOUDFRAME_H
#define SRC_CLOUDFRAME_H

#include "utility.h"


struct Point3D{
    Eigen::Vector3d raw_points;
    Eigen::Vector3d global_points;

    double intensity;
    double alpha_time=0;//[0,1]
    double timestamp=0;
    int ring=0;
    int frame_id=0;

    Point3D()=default;
};

class CloudFrame {

public:
    std::vector<Point3D> points;
    double timeStart, timeEnd;
    pcl::PointCloud<pcl::PointXYZI> pcl_ori;

    CloudFrame();

    void lidarConvert(const sensor_msgs::PointCloud2ConstPtr &laserMsg,lidar_type lidar);

private:
    void RSlidarConvert(const sensor_msgs::PointCloud2ConstPtr &laserMsg);

};


#endif //SRC_CLOUDFRAME_H
