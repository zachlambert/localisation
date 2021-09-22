#ifndef SIM_H
#define SIM_H

#include <iostream>

#include "geometry.h"
#include "robot.h"
#include "sensor.h"
#include "terrain.h"
#include "state_estimator.h"
#include "controller.h"


struct Sim: public sf::Drawable {

    double dt;

    Terrain terrain;
    Robot robot;
    Lidar lidar;
    Target target;

    StateEstimator state_estimator;
    Controller controller;

    int step_number;

    Sim()
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
    }

    void start(double dt)
    {
        this->dt = dt;
        step_number = 0;
    }

    void stepAll()
    {
        while (!step()) {}
    }

    bool step()
    {
        bool increment = true;
        std::cout << "Sim: Step " << step_number << std::endl;
        switch (step_number) {
            case 0:
                robot.stepModel(controller.command, dt);
                break;
            case 1:
                lidar.sample(robot.pose, terrain);
                lidar.sampleLandmarks(robot.pose, terrain);
                break;
            case 2:
                state_estimator.start(controller.command, &lidar.landmarks, &terrain, dt);
                // state_estimator.start(controller.command, &lidar.scan, dt);
                step_number++;
            case 3:
                increment = state_estimator.step();
                break;
            case 4:
                controller.start(robot.pose, target.pose, dt);
                step_number++;
            case 5:
                increment = controller.step();
                break;
            default:
                return true;
        }

        if (increment) step_number++;
        if (step_number == 6) return true;
        return false;
    }

private:
    virtual void draw(sf::RenderTarget& render_target, sf::RenderStates states)const
    {
        render_target.draw(terrain, states);
        render_target.draw(state_estimator, states);
        render_target.draw(controller, states);
        render_target.draw(robot, states);
        render_target.draw(target, states);
    }
};

#endif
