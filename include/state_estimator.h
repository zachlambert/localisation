#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <iostream>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "render_objects.h"


class StateEstimator: public sf::Drawable {
public:
    StateEstimator()
    {
    }

    void start(const Velocity& command, const Eigen::VectorXd& scan, double dt)
    {
        this->command = command;
        this->scan = scan;
        this->dt = dt;
        step_number = 0;
    }

    bool step()
    {
        std::cout << "State estimator: Step " << step_number << std::endl;
        bool increment = true;
        switch (step_number) {
            case 0:
                // Convert ranges -> points and find features if enabled.
                // TODO point_cloud.create(scan);
                break;
            case 1:
                // Align previous scan and new scan to get an edge constraint
            default:
                return true;
        }

        if (increment) step_number++;
        if (step_number == 2) return true;
        return false;
    }

    // Outputs
    Pose pose;
    Eigen::Matrix3d covariance;

    // Render objects
    mutable struct {
        sf::PoseMarker pose_marker;
        // sf::CovMarker cov_marker; TODO
    } to_draw;

protected:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const
    {
        to_draw.pose_marker.setPosition(pose.position().x(), pose.position().y());
        to_draw.pose_marker.setRotation(pose.orientation() * 180/M_PI);
        target.draw(to_draw.pose_marker, states);
    }

    // Inputs
    Velocity command;
    Eigen::VectorXd scan;
    double dt;

    int step_number;
};


#endif
