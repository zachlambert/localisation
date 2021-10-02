#ifndef MATHS_POINT_CLOUD_H
#define MATHS_POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"


struct Point {
    Eigen::Vector2d pos;

    Point(): pos(0, 0) {}
    Point(const Eigen::Vector2d& pos): pos(pos) {}
    Point(double dist, double angle): pos(dist * getDirection(angle)) {}
    double dist()const{ return pos.norm(); }
    double angle()const{ return std::atan2(pos.y(), pos.x()); }
};

class PointCloud {
public:
    Pose pose;
    Pose true_pose; // Used for rendering
    std::vector<Point> points;
    std::vector<Eigen::VectorXd> descriptors;
};

#endif
