#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H

#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "maths/geometry.h"


sf::Transform getRenderTransform(const Eigen::Vector2d& pos);
sf::Transform getRenderTransform(const Pose& pose);

enum class MarkerType {
    CROSS,
    CIRCLE,
    RING,
    SQUARE
};

void addMarker(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    MarkerType type,
    sf::Color color,
    double size);

void addMesh(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    const std::vector<Eigen::Vector2d>& vertices,
    sf::Color color);

void addLinePrimitive(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    sf::Color color);

enum class LineType {
    LINE,
    ARROW
};

void addLine(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    LineType type,
    sf::Color color,
    double width);

void addEllipse(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& origin,
    double width,
    double height,
    double angle,
    sf::Color color);

void addCovarianceEllipse(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const Eigen::Matrix2d& cov,
    double scaling,
    sf::Color color);

void addSegment(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& position,
    double orientation,
    double radius,
    double width,
    sf::Color color);

void addSegmentSlice(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& position,
    double orientation,
    double radius,
    double length,
    double width,
    sf::Color color);

#endif
