#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H

#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"


sf::Vertex createVertex(sf::Color color, const Eigen::Vector2d& pos);

sf::Vertex createVertex(sf::Color color, double x, double y);

void addTriangle(
    sf::VertexArray& vertex_array,
    sf::Color color,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3);

void addQuad(
    sf::VertexArray& vertex_array,
    sf::Color color,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3,
    const Eigen::Vector2d& p4);

void addMesh(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const std::vector<Eigen::Vector2d>& vertices,
    sf::Color color);

void addMarker(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    double thickness,
    sf::Color color);

void addCircle(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double radius,
    sf::Color color,
    size_t n);

void addSquare(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    sf::Color color);

void addArrow(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    double length,
    double line_width,
    double head_width,
    double head_length,
    sf::Color color);

void addRotArrow(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    double angle,
    double distance,
    double line_width,
    double head_width,
    double head_length,
    sf::Color color);

#endif
