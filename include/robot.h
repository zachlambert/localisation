#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <SFML/Graphics.hpp>
#include "geometry.h"

class Robot {
public:
    Pose pose;
    Velocity vel;

    void stepModel(const Velocity& u, double dt)
    {
        vel = u;
        Eigen::Isometry2d dT = twistToTransform(vel * dt).transform();
        pose.setFromTransform(pose.transform() * dT);
    }
};

#endif
