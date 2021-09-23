#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "render_utils.h"


// For simplicity, having a single point type, which
// has fields for all relevant information.
// Descriptors are stored separately to the point cloud.

struct Point {
    Eigen::Vector2d pos;

    Point(): pos(0, 0) {}
    Point(const Eigen::Vector2d& pos): pos(pos) {}
    Point(double dist, double angle): pos(dist * getDirection(angle)) {}
    double dist()const{ return pos.norm(); }
    double angle()const{ return std::atan2(pos.y(), pos.x()); }
};

class PointCloud: public sf::Drawable {
public:
    Pose pose;
    Pose true_pose; // Used for rendering
    std::vector<Point> points;
    std::vector<Eigen::VectorXd> descriptors;

    PointCloud()
    {
        vertex_array.setPrimitiveType(sf::Triangles);
    }

    void setMarkerColor(sf::Color marker_color) { this->marker_color = marker_color; }
    void setMarkerSize(double marker_size) { this->marker_size = marker_size; }
    void setMarkerType(MarkerType marker_type) { this->marker_type = marker_type; }

    void updateVertices()
    {
        vertex_array.clear();
        for (const auto& point: points) {
            addMarker(vertex_array, point.pos, marker_type, marker_color, marker_size);
        }
    }

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const
    {
        sf::Transform transform;
        states.transform *= getRenderTransform(true_pose);
        target.draw(vertex_array, states);
    }

    sf::Color marker_color;
    double marker_size;
    MarkerType marker_type;

    sf::VertexArray vertex_array;
};

#endif
