#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H

#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"


sf::Vertex create_vertex(sf::Color color, const Eigen::Vector2d& pos);

sf::Vertex create_vertex(sf::Color color, double x, double y);

void add_triangle(
    sf::VertexArray& vertex_array,
    sf::Color color,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3);

void add_quad(
    sf::VertexArray& vertex_array,
    sf::Color color,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3,
    const Eigen::Vector2d& p4);

void add_mesh(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const std::vector<Eigen::Vector2d>& vertices,
    sf::Color color);

void add_marker(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    double thickness,
    sf::Color color);

void add_circle(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double radius,
    sf::Color color,
    size_t n);

void add_arrow(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    double length,
    double line_width,
    double head_width,
    double head_length,
    sf::Color color);

void add_rot_arrow(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    double angle,
    double distance,
    double line_width,
    double head_width,
    double head_length,
    sf::Color color);

#endif
