
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
    scan.points.resize(num_points);
}

void Lidar::sample(const Pose& pose, const Terrain &terrain)
{
    scan.pose = pose;

    for (size_t i = 0; i < scan.points.size(); i++) {
        double angle = i*2*M_PI / scan.points.size();
        scan.points[i].dist = terrain.queryIntersection(pose, angle);
        scan.points[i].angle = angle;
        // TODO: Set using sensor model.
    }

    scan.pointsUpdated();
}
