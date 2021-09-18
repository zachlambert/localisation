#ifndef ROBOT_H
#define ROBOT_H

#include <SFML/Graphics.hpp>
#include "geometry.h"

struct Robot {
    Pose pose;

    struct {
        sf::CircleShape body;
        sf::RectangleShape direction;
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
};

#endif
