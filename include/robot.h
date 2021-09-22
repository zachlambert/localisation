#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <SFML/Graphics.hpp>
#include "geometry.h"

class Robot: public sf::Drawable {
public:
    // Data
    Pose pose;
    Velocity vel;

    Robot();
    void stepModel(const Velocity& u, double dt);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    sf::VertexArray vertex_array;
};


class Target: public sf::Drawable {
public:
    Pose pose;

    Target();

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    sf::VertexArray vertex_array;
};

#endif
