#ifndef MATHS_POINT_CLOUD_H
#define MATHS_POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"


// May or may not have a descriptor
struct Point {
    Eigen::Vector2d pos;
    Eigen::VectorXd descriptor;

    Point(): pos(0, 0) {}
    Point(const Eigen::Vector2d& pos, const Eigen::VectorXd& descriptor):
        pos(pos), descriptor(descriptor)
    {}

    double range(const Pose& frame=Pose())const
    { 
        return (pos - frame.position()).norm();
    }

    double angle(const Pose& frame=Pose())const
    {
        Eigen::Vector2d disp = frame.rotation().transpose() * (pos - frame.position());
        return std::atan2(disp.y(), disp.x());
    }

    Eigen::VectorXd state(const Pose& frame=Pose())const {
        Eigen::VectorXd yi;
        yi.resize(2 + descriptor.size());
        yi(0) = range(frame);
        yi(1) = angle(frame);
        yi.tail(descriptor.size()) = descriptor;
        return yi;
    }
};

struct PointCloud {
    Pose frame;
    std::vector<Point> points;
};

// Used for randomly generating landmark features, and sampling fake features
static Eigen::VectorXd randomDescriptor(size_t N)
{
    Eigen::VectorXd v;
    v.resize(N);
    for (size_t i = 0; i < N; i++) {
        v(i) = (double)rand() / RAND_MAX;
    }
    v /= v.sum();
    return v;
}

#endif
