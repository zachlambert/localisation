#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <SFML/Graphics.hpp>
#include "geometry.h"
#include "render_objects.h"

class Robot: public sf::Drawable {
public:
    Robot()
    {
        to_draw.pose_marker.setColor(sf::Color::Red);
        to_draw.velocity_marker.setColor(sf::Color::Magenta);
        vel.angular() = 1;
    }

    // Data
    Pose pose;
    Velocity vel;

    // Render objects
    mutable struct {
        sf::PoseMarker pose_marker;
        sf::VelocityMarker velocity_marker;
    } to_draw;

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

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        to_draw.pose_marker.setPosition(pose.position().x(), pose.position().y());
        to_draw.pose_marker.setRotation(pose.orientation() * 180/M_PI);

        to_draw.velocity_marker.setPosition(pose.position().x(), pose.position().y());
        to_draw.velocity_marker.setRotation(pose.orientation());
        to_draw.velocity_marker.setVelocity(vel);

        target.draw(to_draw.pose_marker, states);
        target.draw(to_draw.velocity_marker, states);
    }
};

#endif
