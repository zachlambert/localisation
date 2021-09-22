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

    Lidar();
    void setScanSize(size_t num_points);
    void sample(const Pose& pose, const Terrain &terrain);
    void sampleLandmarks(const Pose& pose, const Terrain &terrain);
};

#endif
