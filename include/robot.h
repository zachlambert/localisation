#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <SFML/Graphics.hpp>
#include "geometry.h"

struct Robot {
    Pose pose;
    Velocity vel;

    struct {
        sf::CircleShape body;
        sf::RectangleShape direction;
        sf::VertexArray vel_arrow;
    } to_render;

    Robot(double radius, sf::Color color): pose()
    {
        to_render.body.setRadius(radius);
        to_render.body.setFillColor(color);
        to_render.body.setOrigin(radius, radius);

        to_render.direction.setSize(sf::Vector2f(radius*3, radius*0.5));
        to_render.direction.setOrigin(0, to_render.direction.getSize().y/2);
        to_render.direction.setFillColor(color);
    }

    void step_model(const Eigen::Vector2d& target, double dt)
    {
        // Control as if you know the current pose and target exactly.
        // This is just to get the robot moving.
        Eigen::Vector2d disp = target - pose.position();
        disp = pose.get_R().transpose() * disp;
        double e_theta = std::atan2(disp.y(), disp.x());

        double kv = 2;
        double kw = 1;
        double vmax = 0.5;
        double wmax = 3;

        vel.linear().x() = kv * disp.x();
        if (vel.linear().x() < 0) vel.linear().x() = 0;
        if (vel.linear().x() > vmax) vel.linear().x() = vmax;
        vel.linear().y() = 0;

        vel.angular() = kw * e_theta;
        if (vel.angular() < -wmax) vel.angular() = -wmax;
        if (vel.angular() > wmax) vel.angular() = wmax;

        Eigen::Isometry2d dT = twist_to_transform(vel * dt).get_T();
        pose.set_from_T(pose.get_T() * dT);
    }

    void step_state_estimation()
    {

    }
};

#endif
