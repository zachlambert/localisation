#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "terrain.h"
#include "point_cloud.h"


class Lidar {
public:
    PointCloud scan;
    PointCloud landmarks;

    void setScanSize(size_t num_points)
    {
        scan.points.resize(num_points);
    }

    void sample(const Pose& pose, const Terrain &terrain)
    {
        scan.true_pose = pose;

        for (size_t i = 0; i < scan.points.size(); i++) {
            double angle = i*2*M_PI / scan.points.size();
            scan.points[i] = Point(terrain.queryIntersection(pose, angle), angle);
            // TODO: Set using sensor model.
        }
    }

    void sampleLandmarks(const Pose& pose, const Terrain &terrain)
    {
        terrain.getObservableLandmarks(pose, landmarks);
    }

};


#endif
