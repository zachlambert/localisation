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
    StateEstimator(
            const MotionModel& motion_model):
            // const MeasurementModel& measurement_model): TODO
        motion_model(motion_model)
        // measurement_model(measurement_model) TODO
    {
        addStep(&StateEstimator::predict);
        addStep(&StateEstimator::update);
    }

    // General function for getting a state estimate.
    // May store the state estimate explicitly, or may calculate
    // using a parameterisation (eg: particles).
    virtual Pose getStateEstimate()const = 0;

    // Provide the data for a new predict and update step.
    // Needs the following information in general.
    // - Control data (odometry)
    // - Observations (scan)
    // - Known map (terrain) [May or may not be used]
    void start(Velocity* twistEstimate, const PointCloud* scan, const Terrain* terrain)
    {
        this->twistEstimate = twistEstimate;
        this->scan = scan;
        this->terrain = terrain;
        Step::start();
    }

protected:
    virtual bool predict() = 0;
    virtual bool update() = 0;

    // Models
    const MotionModel& motion_model;
    // const MeasurementModel& measurement_model TODO

    // Inputs
    const Velocity* twistEstimate;
    const PointCloud* scan = nullptr;
    const Terrain* terrain = nullptr;
};


class StateEstimatorEKF: public StateEstimator {
public:
    StateEstimateGaussian x;

    StateEstimatorEKF(const MotionModel& motion_model):
        StateEstimator(motion_model)
    {}

    virtual Pose getStateEstimate()const
    {
        return x.pose;
    }

protected:
    virtual bool predict()
    {
        x = motion_model.getGaussian(x, *twistEstimate);
        return true;
    }

    virtual bool update()
    {
        // TODO
        return true;
    }
};

#endif
