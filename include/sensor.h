#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "terrain.h"

class LaserScan {
public:
    struct {
        sf::VertexArray measurements;
    } to_render;

    LaserScan(size_t n, double marker_size=0.1, sf::Color marker_color=sf::Color::Blue):
        n(n),
        marker_size(marker_size),
        marker_color(marker_color)
    {
        y.resize(n);
        y.setZero();
    }
    void sample(const Pose &pose, const Terrain &terrain);

    const Eigen::VectorXd& measurements()const
    {
        return y;
    }
    Eigen::VectorXd& measurements()
    {
        return y;
    }
private:
    size_t n;
    Eigen::VectorXd y;
    double marker_size;
    sf::Color marker_color;
};

#endif
