#ifndef ALGORITHM_CONTROLLER_H
#define ALGORITHM_CONTROLLER_H

#include <Eigen/Core>

#include "maths/geometry.h"
#include "utils/step.h"


class Controller: public Step<Controller> {
public:
    Controller()
    {
        addStep(&Controller::step_main);
    }

    void start(const Pose& pose_estimate)
    {
        this->pose_estimate = pose_estimate;
        Step::start();
    }

    bool step_main()
    {
        Eigen::Vector2d disp = target.position() - pose_estimate.position();
        disp = pose_estimate.rotation().transpose() * disp;
        double e_theta = std::atan2(disp.y(), disp.x());

        double kv = 2;
        double kw = 1;
        double vmax = 0.5;
        double wmax = 3;

        command.linear().x() = kv * disp.x();
        if (command.linear().x() < 0) command.linear().x() = 0;
        if (command.linear().x() > vmax) command.linear().x() = vmax;
        command.linear().y() = 0;

        command.angular() = kw * e_theta;
        if (command.angular() < -wmax) command.angular() = -wmax;
        if (command.angular() > wmax) command.angular() = wmax;

        return true;
    }
    
    void setTarget(const Pose& target) { this->target = target; }
    Pose getTarget()const { return target; }

    // Outputs
    Velocity command;

protected:
    // Inputs
    Pose pose_estimate;
    Pose target;
};
#endif
