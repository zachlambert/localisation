#ifndef ROBOT_H
#define ROBOT_H

#include "maths/geometry.h"
#include "maths/motion_model.h"


class Robot {
public:
    Pose pose;
    Velocity vel;

    Robot(const MotionModel* motion_model):
        motion_model(motion_model)
    {}

    void stepModel(const Velocity& u)
    {
        pose = motion_model->sampleForward(pose, u);
    }

private:
    const MotionModel* motion_model;
};

#endif
