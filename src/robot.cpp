
#include "robot.h"

#include "render_utils.h"


// ===== Robot =====

Robot::Robot()
{
    vertex_array.setPrimitiveType(sf::Triangles);

    const double radius = 0.1;
    const sf::Color color = sf::Color::Red;

    addEllipse(
        vertex_array,
        radius*2, radius*2, 0,
        color);
    addLine(
        vertex_array,
        Eigen::Vector2d(0, 0), Eigen::Vector2d(3*radius, 0),
        LineType::ARROW,
        color,
        radius*0.5);
}


void Robot::stepModel(const Velocity& u, double dt)
{
    vel = u;
    Eigen::Isometry2d dT = twistToTransform(vel * dt).transform();
    pose.setFromTransform(pose.transform() * dT);
}


void Robot::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    states.transform *= getRenderTransform(pose);
    target.draw(vertex_array, states);
}


// ===== Target =====

Target::Target()
{
    vertex_array.setPrimitiveType(sf::Triangles);

    sf::Color color = sf::Color::Red;
    double size = 0.2;

    addMarker(
        vertex_array,
        Eigen::Vector2d(0,0),
        MarkerType::CROSS,
        color,
        size);
}

void Target::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    states.transform *= getRenderTransform(pose);
    target.draw(vertex_array, states);
}
