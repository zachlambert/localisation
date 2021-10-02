#ifndef MATHS_POINT_CLOUD_H
#define MATHS_POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"


// May or may not have a descriptor
struct Point {
    double range;
    double angle;
    Eigen::VectorXd descriptor;

    Point(): range(0), angle(0) {}
    Point(double range, double angle): range(range), angle(angle) {}
    Point(double range, double angle, const Eigen::VectorXd& descriptor):
        range(range), angle(angle), descriptor(descriptor)
    {}

    Eigen::Vector2d pos()const { return range * getDirection(angle); }

    Eigen::VectorXd state()const {
        Eigen::VectorXd yi;
        yi.resize(2 + descriptor.size());
        yi(0) = range;
        yi(1) = angle;
        yi.tail(descriptor.size()) = descriptor;
        return yi;
    }
};

struct PointCloud {
    std::vector<Point> points;
    // May need additional information later
};

Eigen::VectorXd getPointInnovation(const Point& y, const Point& y_predicted)
{
    Eigen::VectorXd dif = y.state() - y_predicted.state();
    normaliseAngle(dif(1));
    return dif;
}

#endif
