#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "point_cloud.h"
#include "terrain.h"
#include "step.h"

class StateEstimator: public Step<StateEstimator> {
public:
    // Outputs
    Pose pose;
    Eigen::Matrix3d covariance;

    StateEstimator()
    {
        addStep(&StateEstimator::predict);
        addStep(&StateEstimator::update);
    }

    void start(const Velocity& command, const PointCloud* scan, const Terrain* terrain, double dt)
    {
        this->command = command;
        this->scan = scan;
        this->terrain = terrain;
        this->dt = dt;
        Step::start();
    }

    bool predict()
    {
        Velocity twist = command*dt;
        twist.linear().x() += 0.1*dt; // TODO sample noise properly
        twist.linear().y() -= 0.2*dt;
        twist.angular() += 0.1*dt;
        pose.setFromTransform(pose.transform() * twistToTransform(twist).transform());
        return true;
    }

    bool update()
    {
        return true;
    }

protected:

    // Inputs
    Velocity command;
    const PointCloud* scan = nullptr;
    const Terrain* terrain = nullptr;
    double dt;
};


#endif
