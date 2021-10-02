#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "maths/measurement_model.h"

class Lidar {
public:
    PointCloud ranges;
    PointCloud features;

    Lidar(const RangeModel& range_model, const FeatureModel& feature_model):
        range_model(range_model), feature_model(feature_model)
    {}

    void setScanSize(size_t num_points)
    {
        ranges.points.resize(num_points);
    }

    void sampleRanges(const Pose& pose, const Terrain &terrain)
    {
        for (size_t i = 0; i < ranges.points.size(); i++) {
            double angle = i*2*M_PI / ranges.points.size();
            ranges.points[i].range = range_model.sample(angle, pose, terrain);
            ranges.points[i].angle = angle;
        }
    }

    void sampleFeatures(const Pose& pose, const Terrain &terrain)
    {
        feature_model.sample(features, pose, terrain);
    }

private:
    const RangeModel& range_model;
    const FeatureModel& feature_model;
};

#endif
