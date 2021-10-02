#ifndef MATHS_POINT_CLOUD_H
#define MATHS_POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"


struct Point {
    double range;
    double angle;
    Eigen::VectorXd descriptor;

    Point(): range(0), angle(0), descriptor() {}
    Point(double range, double angle): range(range)

    double dist()const{ return pos.norm(); }
    double angle()const{ return std::atan2(pos.y(), pos.x()); }

    void setPolar(double dist, double angle) {
        pos = dist * getDirection(angle);
    }

    Eigen::VectorXd getState()const {
        Eigen::VectorXd yi;
        yi.resize(2 + descriptor.size());
        yi(0) = dist();
        yi(1) = points[index].angle();
        yi.tail(descriptors[index].size()) = descriptors[index];
        return yi;
    }
};

class PointCloud {
public:
    Pose pose;
    Pose true_pose; // Used for rendering
    std::vector<Point> points;
};

Eigen::VectorXd getPointInnovation(const PointCloud& 
{
    auto dif = k - mean;
}

#endif
