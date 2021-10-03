#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "state/robot.h"
#include "maths/measurement_model.h"

class RangeSensor {
public:
    PointCloud ranges;
    PointCloud features;

    RangeSensor(const RangeModel& range_model):
        range_model(range_model)
    {}

    void setScanSize(size_t num_points)
    {
        ranges.points.resize(num_points);
    }

    void sample(const Terrain& terrain, const Robot& robot)
    {
        for (size_t i = 0; i < ranges.points.size(); i++) {
            double angle = i*2*M_PI / ranges.points.size();
            ranges.points[i].range = range_model.sample(angle, robot.pose, terrain);
            ranges.points[i].angle = angle;
        }
    }

private:
    const RangeModel& range_model;
};

#endif
