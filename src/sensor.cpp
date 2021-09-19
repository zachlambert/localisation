
#include <iostream>
#include "sensor.h"
#include "render_utils.h"


void LaserScan::sample(const Terrain &terrain)
{
    for (size_t i = 0; i < y.size(); i++) {
        y(i) = terrain.query_intersection(pose, i*2*M_PI / y.size());
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
