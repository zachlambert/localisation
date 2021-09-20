#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "terrain.h"
#include "render_objects.h"


class Lidar: public sf::Drawable {
public:
    Eigen::VectorXd scan;

    mutable struct {
        sf::MarkerArray measurements;
    } to_draw;

    Lidar();
    void setNumPoints(size_t num_points);
    void sample(const Pose& pose, const Terrain &terrain);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    Pose pose;
};

#endif
