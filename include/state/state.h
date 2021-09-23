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


struct State: public Step<State> {

    double dt;

    MotionModel motion_model;

    Terrain terrain;
    Robot robot;
    Lidar lidar;
    Pose target;

    StateEstimator state_estimator;
    Controller controller;

    State():
        robot(&motion_model),
        state_estimator(&motion_model)
    {
        motion_model.setTwistVarianceScaling(0.01, 0.01, 0.01);

        createTerrain(terrain);

        robot.pose.position() = Eigen::Vector2d(-3, 3);
        robot.pose.orientation() = 0.5;
        robot.vel.linear() = Eigen::Vector2d(1, 0.2);
        robot.vel.angular() = 1;

        state_estimator.pose = robot.pose;
        state_estimator.covariance = Eigen::Vector3d(0.01, 0.01, 0.1).asDiagonal();

        lidar.setScanSize(100);

        target.position() = Eigen::Vector2d(2, 2);

        addStep(&State::step_motion);
        addStep(&State::step_lidar);
        addStep(&State::step_state_estimator);
        addStep(&State::step_controller);
    }

    void start(double dt)
    {
        this->dt = dt;
        step_number = 0;
        motion_model.setTimeStep(dt);
        Step::start();
    }

    bool step_motion()
    {
        robot.stepModel(controller.command);
        return true;
    }

    bool step_lidar()
    {
        lidar.sample(robot.pose, terrain);
        lidar.sampleLandmarks(robot.pose, terrain);
        return true;
    }

    bool step_state_estimator()
    {
        if (!state_estimator.started()) {
            state_estimator.start(
                controller.command, &lidar.landmarks, &terrain, dt
            );
        }
        return state_estimator.step();
    }

    bool step_controller()
    {
        if (!controller.started()) {
            controller.start(state_estimator.pose, target, dt);
        }
        return controller.step();
    }
};

#endif
