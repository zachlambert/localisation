
#include "robot.h"


// ===== Robot =====

Robot::Robot()
{
    to_draw.pose_marker.setColor(sf::Color::Red);
    to_draw.velocity_marker.setColor(sf::Color::Magenta);
    vel.angular() = 1;
    to_draw.enable_velocity_marker = false;
}


void Robot::step_model(const Eigen::Vector2d& target, double dt)
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
    to_draw.marker.setPosition(position.x(), position.y());
    target.draw(to_draw.marker, states);
}