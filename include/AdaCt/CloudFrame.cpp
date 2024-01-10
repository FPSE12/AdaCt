//
// Created by wjj on 2024/1/6.
//

#include "CloudFrame.h"

CloudFrame::CloudFrame(){

}

void CloudFrame::lidarConvert(const sensor_msgs::PointCloud2ConstPtr &laserMsg,lidar_type lidar) {
    RSlidarConvert(laserMsg);
}


void CloudFrame::RSlidarConvert(const sensor_msgs::PointCloud2ConstPtr &laserMsg) {

}