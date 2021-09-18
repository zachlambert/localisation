
#include <iostream>
#include "sensor.h"
#include "render_utils.h"


void LaserScan::sample(const Pose &pose, const Terrain &terrain)
{
    for (size_t i = 0; i < y.size(); i++) {
        double angle = i*2*M_PI/n;
        y(i) = terrain.query_intersection(pose, i*2*M_PI / n);
        // TODO: Set using sensor model.
    }

    // Update to_render
    to_render.measurements.setPrimitiveType(sf::Triangles);
    to_render.measurements.clear();
    for (size_t i = 0; i < y.size(); i++) {
        double angle = i*2*M_PI/n;
        add_marker(
            to_render.measurements,
            y(i) * Eigen::Vector2d(std::cos(angle), std::sin(angle)),
            marker_size,
            marker_color
        );
    }
}
