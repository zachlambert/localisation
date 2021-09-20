#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"


class Controller: public sf::Drawable {
public:
    Controller() {}

    void start(const Pose& pose, const Pose& target, double dt)
    {
        this->pose = pose;
        this->target = target;
        this->dt = dt;
    }

    bool step()
    {
        std::cout << "Controller: Step 0" << std::endl;
        // Control as if you know the current pose and target exactly.
        // This is just to get the robot moving.

        Eigen::Vector2d disp = target.position() - pose.position();
        disp = pose.rotation().transpose() * disp;
        double e_theta = std::atan2(disp.y(), disp.x());

        double kv = 2;
        double kw = 1;
        double vmax = 0.5;
        double wmax = 3;

        command.linear().x() = kv * disp.x();
        if (command.linear().x() < 0) command.linear().x() = 0;
        if (command.linear().x() > vmax) command.linear().x() = vmax;
        command.linear().y() = 0;

        command.angular() = kw * e_theta;
        if (command.angular() < -wmax) command.angular() = -wmax;
        if (command.angular() > wmax) command.angular() = wmax;

        return true;
    }

    // Outputs
    Velocity command;

protected:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const
    {
        // Nothing
    }

    double dt;

    // Inputs
    Pose pose;
    Pose target;
};
#endif
