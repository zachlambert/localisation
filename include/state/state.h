#ifndef SIM_H
#define SIM_H

#include "algorithm/controller.h"
#include "algorithm/state_estimator.h"
#include "maths/geometry.h"
#include "maths/motion_model.h"
#include "state/robot.h"
#include "state/sensor.h"
#include "state/terrain.h"
#include "utils/step.h"

#include <iostream>


struct State: public Step<State> {

    double dt;

    Terrain terrain;
    Robot robot;
    Lidar lidar;
    Pose target;

    StateEstimator& state_estimator;
    Controller& controller;

    State(  const MotionModel& motion_model,
            const RangeModel& range_model,
            const FeatureModel& feature_model,
            StateEstimator& state_estimator,
            Controller& controller):
        robot(motion_model),
        lidar(range_model, feature_model),
        state_estimator(state_estimator),
        controller(controller)
    {
        createTerrain(terrain);

        robot.pose.position() = Eigen::Vector2d(-3, 3);
        robot.pose.orientation() = 0.5;

        state_estimator.resetEstimate(robot.pose);

        lidar.setScanSize(100);

        target.position() = Eigen::Vector2d(2, 2);

        addStep(&State::stepModel);
        addStep(&State::stepStateEstimator);
        addStep(&State::stepController);
    }

    void start(double dt)
    {
        this->dt = dt;
        step_number = 0;
        Step::start();
    }

    bool stepModel()
    {
        robot.stepModel(controller.command, dt);
        lidar.sampleRanges(robot.pose, terrain);
        lidar.sampleFeatures(robot.pose, terrain);
        return true;
    }

    bool stepStateEstimator()
    {
        if (!state_estimator.started()) {
            state_estimator.start(
                &robot.twistEstimate,
                &lidar.features,
                &terrain
            );
        }
        return state_estimator.step();
    }

    bool stepController()
    {
        if (!controller.started()) {
            controller.start(robot.pose, target, dt);
            // controller.start(state_estimator.getStateEstimate(), target, dt);
        }
        return controller.step();
    }
};

#endif
