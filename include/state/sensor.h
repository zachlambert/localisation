#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"


class Lidar {
public:
    PointCloud scan;
    PointCloud landmarks;

    Lidar(const MeasurementModel& range_measurement_model

    void setScanSize(size_t num_points)
    {
        scan.points.resize(num_points);
    }

    void sampleRanges(const Pose& pose, const Terrain &terrain)
    {
        scan.true_pose = pose;

        for (size_t i = 0; i < scan.points.size(); i++) {
            double angle = i*2*M_PI / scan.points.size();
            double true_dist = terrain.queryIntersection(pose, angle);
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
