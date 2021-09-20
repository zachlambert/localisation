
#include <iostream>
#include "sensor.h"
#include "render_utils.h"


// ===== Lidar =====

Lidar::Lidar()
{
    setNumPoints(20);
    to_draw.measurements.setColor(sf::Color::Blue);
    to_draw.measurements.setSize(0.1);
    to_draw.measurements.setThickness(0.02);
}

void Lidar::setNumPoints(size_t num_points)
{
    scan.resize(num_points);
    scan.setZero();
}

void Lidar::sample(const Pose& pose, const Terrain &terrain)
{
    this->pose = pose;

    for (size_t i = 0; i < scan.size(); i++) {
        scan(i) = terrain.queryIntersection(pose, i*2*M_PI / scan.size());
        // TODO: Set using sensor model.
    }

    to_draw.measurements.clearMarkers();
    for (size_t i = 0; i < scan.size(); i++) {
        to_draw.measurements.addMarker(
            scan(i) * get_direction(pose.orientation() + (i*2*M_PI/scan.size()))
        );
    }
}

void Lidar::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    to_draw.measurements.setPosition(pose.position().x(), pose.position().y());
    target.draw(to_draw.measurements, states);
}
