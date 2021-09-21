#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"

// For simplicity, having a single point type, which
// has fields for all relevant information.
// Descriptors are stored separately to the point cloud.

struct Point {
    Eigen::Vector2d pos;
    double dist;
    double angle;
    double intensity;
};

class PointCloud {
public:
    Pose pose;
    std::vector<Point> points;
    std::vector<Eigen::VectorXd> descriptors;

    PointCloud()
    {
        vertex_array.setPrimitiveType(sf::Triangles);
        dirty = true;
    }

    // Could put points behind accessor function which set dirty=true
    // but the consistency of points being publish with pose and descriptors
    // outweights the drawback of calling pointsUpdated() to update render
    // information.
    void pointsUpdated()
    {
        dirty = true;
    }

    enum class MarkerType {
        CROSS,
        CIRCLE,
        RING,
        SQUARE
    };

    void setMarkerColor(sf::Color marker_color)
    {
        this->marker_color = marker_color;
        dirty = true;
    }

    void setMarkerSize(double marker_size)
    {
        this->marker_size = marker_size;
        dirty = true;
    }

    void setMarkerType(MarkerType marker_type)
    {
        this->marker_type = marker_type;
        dirty = true;
    }

private:
    void updateVertices()const;
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    sf::Color marker_color;
    double marker_size;
    MarkerType marker_type;

    mutable bool dirty;
    mutable sf::VertexArray vertex_array;
};

#endif
