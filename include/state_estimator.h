#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "point_cloud.h"
#include "terrain.h"


class StateEstimator: public sf::Drawable {
public:
    StateEstimator():
        scan(nullptr)
    {
        vertex_array.setPrimitiveType(sf::Triangles);
    }

    void start(const Velocity& command, const PointCloud* scan, const Terrain* terrain, double dt)
    {
        this->command = command;
        this->scan = scan;
        this->terrain = terrain;
        this->dt = dt;
        step_number = 0;
    }

    bool step()
    {
        std::cout << "State estimator: Step " << step_number << std::endl;
        bool increment = true;
        switch (step_number) {
            case 0:
                predict();
                break;
            case 1:
                update();
                break;
            default:
                return true;
        }

        updateVertices();

        if (increment) step_number++;
        if (step_number == 2) return true;
        return false;
    }

    void predict()
    {
        Velocity twist = command*dt;
        twist.linear().x() += 0.1*dt; // TODO sample noise properly
        twist.linear().y() -= 0.2*dt;
        twist.angular() += 0.1*dt;
        pose.setFromTransform(pose.transform() * twistToTransform(twist).transform());
    }

    void update()
    {

    }

    // Outputs
    Pose pose;
    Eigen::Matrix3d covariance;

    void updateVertices()
    {
        vertex_array.clear();

        const double cov_position_scaling = 40;
        const double cov_orientation_scaling = 10;
        addCovarianceEllipse(
            vertex_array,
            covariance.block<2,2>(0,0),
            cov_position_scaling,
            sf::Color::Green);
        addSegment(
            vertex_array,
            0.5,
            0,
            cov_orientation_scaling * covariance(2,2),
            sf::Color::Blue);

        const double radius = 0.05;
        addEllipse(
            vertex_array,
            2*radius,
            2*radius,
            0, // orientation
            sf::Color::Black);
        addLine(
            vertex_array,
            Eigen::Vector2d(0,0),
            Eigen::Vector2d(5*radius, 0),
            LineType::ARROW,
            sf::Color::Black,
            radius);
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
    const Terrain* terrain;
    double dt;

    int step_number;

    sf::VertexArray vertex_array;
};


#endif
