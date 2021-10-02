#ifndef ROBOT_H
#define ROBOT_H

#include "maths/geometry.h"
#include "maths/motion_model.h"


class Robot {
public:
    Pose pose;

    Robot(const MotionModel& motion_model):
        motion_model(motion_model)
    {}

    void stepModel(const Velocity& u, double dt)
    {
        // Update the odometry model with the incremental transform
        // for the current step.
        odometry.update(twistToTransform(u * dt));
        // Sample a possible next step from the motion model,
        // which uses this odometry.
        pose = motion_model.sample(pose, odometry);
    }

    const Odometry* getOdometry()const {
        return &odometry;
    }

private:
    Odometry odometry;
    const MotionModel& motion_model;
};

#endif
