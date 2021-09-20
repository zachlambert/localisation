#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <SFML/Graphics.hpp>
#include "geometry.h"
#include "render_objects.h"

class Robot: public sf::Drawable {
public:
    // Data
    Pose pose;
    Velocity vel;

    // Render objects
    mutable struct {
        sf::PoseMarker pose_marker;
        sf::VelocityMarker velocity_marker;
        bool enable_velocity_marker;
    } to_draw;

    Robot();
    void stepModel(const Velocity& u, double dt);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;
};


class Target: public sf::Drawable {
public:
    Pose pose;

    mutable struct {
        sf::Marker marker;
    } to_draw;

    Target();

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;
};

#endif
