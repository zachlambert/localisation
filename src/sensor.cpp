
#include <iostream>
#include "sensor.h"
#include "render_utils.h"


// ===== LaserScan =====

LaserScan::LaserScan()
{
    setNumPoints(20);
    to_draw.measurements.setColor(sf::Color::Blue);
    to_draw.measurements.setSize(0.1);
    to_draw.measurements.setThickness(0.02);
}

void LaserScan::setNumPoints(size_t num_points)
{
    y.resize(num_points);
    y.setZero();
}

void LaserScan::sample(const Terrain &terrain)
{
    for (size_t i = 0; i < y.size(); i++) {
        y(i) = terrain.queryIntersection(pose, i*2*M_PI / y.size());
        // TODO: Set using sensor model.
    }

    to_draw.measurements.clearMarkers();
    for (size_t i = 0; i < y.size(); i++) {
        to_draw.measurements.addMarker(
            y(i) * get_direction(pose.orientation() + (i*2*M_PI/y.size()))
        );
    }
}

void LaserScan::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    to_draw.measurements.setPosition(pose.position().x(), pose.position().y());
    target.draw(to_draw.measurements, states);
}
