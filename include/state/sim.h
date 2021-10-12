#ifndef STATE_SIM_H
#define STATE_SIM_H

#include "state/terrain.h"
#include "state/robot.h"
#include "state/sensor.h"


class Sim {
public:
    Terrain terrain;
    Robot robot;
    Sensor sensor;

    Sim(const MotionModel& motion_model, const MeasurementModel& measurement_model):
        robot(motion_model),
        sensor(measurement_model)
    {}

    void step(double dt, const Velocity& robot_command)
    {
        robot.step(robot_command, dt);
        sensor.sample(terrain, robot);
    }
};

#endif
