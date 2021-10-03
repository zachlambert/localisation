#ifndef ROBOT_H
#define ROBOT_H

#include "maths/geometry.h"
#include "maths/motion_model.h"


class Robot {
public:
    Pose pose;
    Velocity twistEstimate; // Measurement

    Robot(const MotionModel& motion_model):
        motion_model(motion_model)
    {}

    void step(const Velocity& target_u, double dt)
    {
        Velocity twist = target_u * dt;
        pose = motion_model.sample(pose, twist);

        // Could add a random error between target velocity and actual
        // mean velocity, but not important. The motion model adds velocity
        // variance throughout interval anyway.
        twistEstimate = twist;
    }

private:
    const MotionModel& motion_model;
};

#endif
