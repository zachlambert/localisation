#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <SFML/Graphics.hpp>
#include "geometry.h"
#include "render_objects.h"

class Robot: public sf::Drawable {
public:
    Robot();

    // Data
    Pose pose;
    Velocity vel;

    // Render objects
    mutable struct {
        sf::PoseMarker pose_marker;
        sf::VelocityMarker velocity_marker;
        bool enable_velocity_marker;
    } to_draw;

    void step_model(const Eigen::Vector2d& target, double dt);
    void step_state_estimation();

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;
};


class Target: public sf::Drawable {
public:
    Target();

    // Data
    Eigen::Vector2d position;

    // Render objects
    mutable struct {
        sf::Marker marker;
    } to_draw;

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;
};

#endif
