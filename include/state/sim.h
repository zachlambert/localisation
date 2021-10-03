#ifndef STATE_SIM_H
#define STATE_SIM_H

#include "state/terrain.h"
#include "state/robot.h"
#include "state/sensor.h"


class Sim {
public:
    Terrain terrain;
    Robot robot;
    RangeSensor range_sensor;

    Sim(const MotionModel& motion_model, const RangeModel& range_model):
        robot(motion_model),
        range_sensor(range_model)
    {}
    void step(double dt, const Velocity& robot_command)
    {
        robot.step(robot_command, dt);
        range_sensor.sample(terrain, robot);
    }
};

#endif
