#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "state/robot.h"
#include "maths/measurement_model.h"

class Sensor {
public:
    PointCloud ranges;

    Sensor(const MeasurementModel& measurement_model):
        measurement_model(measurement_model)
    {}

    void setScanSize(size_t num_points)
    {
        ranges.points.resize(num_points);
    }

    void sample(const Terrain& terrain, const Robot& robot)
    {
        for (size_t i = 0; i < ranges.points.size(); i++) {
            double angle = i*2*M_PI / ranges.points.size();
            double range = measurement_model.sample(robot.pose, angle, terrain);
            ranges.points[i].setPolar(range, angle);
        }
    }

private:
    const MeasurementModel& measurement_model;
};

#endif
