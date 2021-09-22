#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "point_cloud.h"


class StateEstimator: public sf::Drawable {
public:
    StateEstimator():
        scan(nullptr)
    {
        vertex_array.setPrimitiveType(sf::Triangles);
    }

    void start(const Velocity& command, const PointCloud* scan, double dt)
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

        updateVertices();

        if (increment) step_number++;
        if (step_number == 2) return true;
        return false;
    }

    // Outputs
    Pose pose;
    Eigen::Matrix3d covariance;

    void updateVertices()
    {
        vertex_array.clear();

        const double radius = 0.1;
        addEllipse(
            vertex_array,
            2*radius,
            2*radius,
            0, // orientation
            sf::Color::Black);
        addLine(
            vertex_array,
            Eigen::Vector2d(0,0),
            Eigen::Vector2d(3*radius, 0),
            LineType::ARROW,
            sf::Color::Black,
            0.5*radius);

        const double cov_position_scaling = 1;
        const double cov_orientation_scaling = 1;
        addCovarianceEllipse(
            vertex_array,
            covariance.block<2,2>(0,0),
            cov_position_scaling,
            sf::Color::Green);
        addSegment(
            vertex_array,
            0.5,
            pose.orientation(),
            cov_orientation_scaling * covariance(2,2),
            sf::Color::Cyan);
    }

protected:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const
    {
        // Before transform
        if (scan != nullptr) target.draw(*scan, states);

        // Apply transform to pose estimate
        states.transform *= getRenderTransform(pose);
        target.draw(vertex_array, states);
    }

    // Inputs
    Velocity command;
    const PointCloud* scan;
    double dt;

    int step_number;

    sf::VertexArray vertex_array;
};


#endif
