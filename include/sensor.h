#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "terrain.h"
#include "render_objects.h"


class LaserScan: public sf::Drawable {
public:
    LaserScan()
    {
        setNumPoints(20);
        to_draw.measurements.setColor(sf::Color::Blue);
        to_draw.measurements.setSize(0.1);
        to_draw.measurements.setThickness(0.02);
    }

    // Data
    Eigen::VectorXd y;
    Pose pose;

    // Render objects
    mutable struct {
        sf::MarkerArray measurements;
    } to_draw;


    void setNumPoints(size_t num_points)
    {
        y.resize(num_points);
        y.setZero();
    }

    void sample(const Terrain &terrain);

    const Eigen::VectorXd& measurements()const
    {
        return y;
    }
    Eigen::VectorXd& measurements()
    {
        return y;
    }

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

};

#endif
