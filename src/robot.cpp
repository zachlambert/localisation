
#include "robot.h"


// ===== Robot =====

Robot::Robot()
{
    to_draw.pose_marker.setColor(sf::Color::Red);
    to_draw.velocity_marker.setColor(sf::Color::Magenta);
    vel.angular() = 1;
    to_draw.enable_velocity_marker = false;
}


void Robot::stepModel(const Velocity& u, double dt)
{
    vel = u;
    Eigen::Isometry2d dT = twistToTransform(vel * dt).transform();
    pose.setFromTransform(pose.transform() * dT);
}


void Robot::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    to_draw.pose_marker.setPosition(pose.position().x(), pose.position().y());
    to_draw.pose_marker.setRotation(pose.orientation() * 180/M_PI);

    to_draw.velocity_marker.setPosition(pose.position().x(), pose.position().y());
    to_draw.velocity_marker.setRotation(pose.orientation() * 180/M_PI);
    to_draw.velocity_marker.setVelocity(vel);

    target.draw(to_draw.pose_marker, states);
    if (to_draw.enable_velocity_marker) {
        target.draw(to_draw.velocity_marker, states);
    }
}


// ===== Target =====

Target::Target()
{
    to_draw.marker.setSize(0.5);
    to_draw.marker.setThickness(0.1);
}

void Target::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    to_draw.marker.setPosition(pose.position().x(), pose.position().y());
    target.draw(to_draw.marker, states);
}
