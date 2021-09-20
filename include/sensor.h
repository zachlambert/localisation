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
    Eigen::VectorXd y;
    Pose pose;

    mutable struct {
        sf::MarkerArray measurements;
    } to_draw;

    LaserScan();
    void setNumPoints(size_t num_points);
    void sample(const Terrain &terrain);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

};

#endif
