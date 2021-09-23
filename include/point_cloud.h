#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "render_utils.h"


// For simplicity, having a single point type, which
// has fields for all relevant information.
// Descriptors are stored separately to the point cloud.

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
