
#include <iostream>
#include "sensor.h"
#include "render_utils.h"


// ===== Lidar =====

Lidar::Lidar()
{
    setScanSize(20);
    scan.setMarkerColor(sf::Color::Blue);
    scan.setMarkerType(MarkerType::CIRCLE);
    scan.setMarkerSize(0.04);

    landmarks.setMarkerColor(sf::Color::Magenta);
    landmarks.setMarkerType(MarkerType::RING);
    landmarks.setMarkerSize(0.5);
}

void Lidar::setScanSize(size_t num_points)
{
    scan.points.resize(num_points);
}

void Lidar::sample(const Pose& pose, const Terrain &terrain)
{
    scan.true_pose = pose;

    for (size_t i = 0; i < scan.points.size(); i++) {
        double angle = i*2*M_PI / scan.points.size();
        scan.points[i] = Point(terrain.queryIntersection(pose, angle), angle);
        // TODO: Set using sensor model.
    }

    scan.updateVertices();
}

void Lidar::sampleLandmarks(const Pose& pose, const Terrain &terrain)
{
    terrain.getObservableLandmarks(pose, landmarks);
    // TODO: Add noise

    landmarks.updateVertices();
}
