//
// Created by wjj on 2024/1/4.
//

#ifndef ADACT_POINTTYPE_H
#define ADACT_POINTTYPE_H

#include "utility.h"

struct Point3D{
    Eigen::Vector3d raw_points;
    Eigen::Vector3d global_points;

    double intensity;
    double alpha_time;//[0,1]
    double timestamp;
};

#endif //ADACT_POINTTYPE_H
