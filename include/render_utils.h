#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H

#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"

void add_mesh(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const std::vector<Eigen::Vector2d>& vertices,
    sf::Color color);

void add_marker(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    sf::Color color);

#endif
