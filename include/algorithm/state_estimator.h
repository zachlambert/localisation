#ifndef ALGORITHM_STATE_ESTIMATOR_H
#define ALGORITHM_STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "maths/geometry.h"
#include "maths/motion_model.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "utils/step.h"


class StateEstimatorEKF: public Step<StateEstimatorEKF> {
public:
    StateEstimatorEKF(
            const MotionModel& motion_model):
            // const MeasurementModel& measurement_model): TODO
        motion_model(motion_model)
        // measurement_model(measurement_model) TODO
    {
        addStep(&StateEstimatorEKF::predict);
        addStep(&StateEstimatorEKF::update);
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
    void start(const Odometry* odometry, const PointCloud* scan, const Terrain* terrain, double dt)
    {
        this->odometry = odometry;
        this->scan = scan;
        this->terrain = terrain;
        this->dt = dt;
        Step::start();
    }

private:
    bool predict()
    {
        state_estimate = motion_model.getGaussian(state_estimate, *odometry);
        return true;
    }

    bool update()
    {
        // TODO
        return true;
    }

    // Models
    const MotionModel& motion_model;
    // const MeasurementModel& measurement_model TODO

    // Inputs
    const Odometry* odometry = nullptr;
    const PointCloud* scan = nullptr;
    const Terrain* terrain = nullptr;
    double dt;

    // State estimate
    StateEstimateGaussian state_estimate;
};


#endif
