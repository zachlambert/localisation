#ifndef ALGORITHM_STATE_ESTIMATOR_H
#define ALGORITHM_STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "maths/geometry.h"
#include "maths/motion_model.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "utils/step.h"


class StateEstimator: public Step<StateEstimator> {
public:
    // Outputs
    Pose pose;
    Eigen::Matrix3d covariance;

    StateEstimator(const MotionModel* motion_model):
        motion_model(motion_model)
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
        pose.setFromTransform(pose.transform() * twistToTransform(twist).transform());
        return true;
    }

    bool update()
    {
        return true;
    }

protected:
    const MotionModel* motion_model;

    // Inputs
    Velocity command;
    const PointCloud* scan = nullptr;
    const Terrain* terrain = nullptr;
    double dt;
};


#endif
