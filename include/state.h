#ifndef SIM_H
#define SIM_H

#include <iostream>

#include "geometry.h"
#include "robot.h"
#include "sensor.h"
#include "terrain.h"
#include "state_estimator.h"
#include "controller.h"

#include "step.h"


struct State: public Step<State> {

    double dt;

    Terrain terrain;
    Robot robot;
    Lidar lidar;
    Target target;

    StateEstimator state_estimator;
    Controller controller;

    State()
    {
        createTerrain(terrain);

        robot.pose.position() = Eigen::Vector2d(-3, 3);
        robot.pose.orientation() = 0.5;
        robot.vel.linear() = Eigen::Vector2d(1, 0.2);
        robot.vel.angular() = 1;

        state_estimator.pose = robot.pose;
        state_estimator.covariance = Eigen::Vector3d(0.01, 0.01, 0.1).asDiagonal();

        lidar.setScanSize(100);

        Target target;
        target.pose.position() = Eigen::Vector2d(0, 0);

        addStep(&State::step_motion);
        addStep(&State::step_lidar);
        addStep(&State::step_state_estimator);
        addStep(&State::step_controller);
    }

    void start(double dt)
    {
        this->dt = dt;
        step_number = 0;
        Step::start();
    }

    bool step_motion()
    {
        robot.stepModel(controller.command, dt);
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
        state_estimator.start(controller.command, &lidar.landmarks, &terrain, dt);
        state_estimator.step();
        state_estimator.step();
        return true;
    }

    bool step_controller()
    {
        controller.start(robot.pose, target.pose, dt);
        controller.step();
        return true;
    }
};

#endif
