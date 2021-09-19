#ifndef RENDER_OBJECTS_H
#define RENDER_OBJECTS_H

#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"

namespace sf {

class Arrow: public sf::Drawable, public sf::Transformable {
public:
    Arrow();
    void setLength(double ength);
    void setLineWidth(double line_width);
    void setHeadWidth(double head_width);
    void setHeadLength(double head_length);
    void setFillColor(sf::Color color);

private:
    void update_vertices()const;
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    double length;
    double line_width;
    double head_width;
    double head_length;
    sf::Color color;

    mutable bool dirty;
    mutable sf::VertexArray vertex_array;
};


class RotArrow: public sf::Drawable, public sf::Transformable {
public:
    RotArrow();
    void setAngle(double angle);
    void setDistance(double distance);
    void setLineWidth(double line_width);
    void setHeadWidth(double head_width);
    void setHeadLength(double head_length);
    void setFillColor(sf::Color color);

private:
    void update_vertices()const;
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    double angle;
    double distance;
    double line_width;
    double head_width;
    double head_length;
    sf::Color color;

    mutable bool dirty;
    mutable sf::VertexArray vertex_array;
};


class PoseMarker: public sf::Drawable, public sf::Transformable {
public:
    PoseMarker();
    void setRadius(double radius);
    void setColor(sf::Color color);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

    sf::CircleShape circle;
    sf::Arrow arrow;
};


class VelocityMarker: public sf::Drawable, public sf::Transformable {
public:
    VelocityMarker();
    void setLineWidth(double line_width);
    void setLinearScale(double linear_scale);
    void setAngularScale(double angular_scale);
    void setColor(sf::Color color);
    void setVelocity(const Velocity& velocity);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

    double line_width;
    double linear_scale;
    double angular_scale;
    sf::Color color;
    Velocity velocity;

    sf::Arrow linear_arrow;
    sf::RotArrow angular_arrow;
};

class Marker: public sf::Drawable, public sf::Transformable {
public:
    Marker();
    void setSize(double size);
    void setThickness(double thickness);
    void setColor(sf::Color color);

private:
    void update_vertices()const;
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

    double size;
    double thickness;
    sf::Color color;

    mutable bool dirty;
    mutable sf::VertexArray vertex_array;
};


/*

class MarkerArray: public sf::Drawable, public sf::Transformable {
public:
    MarkerArray():
        size(0.1),
        thickness(0.02)
    {
        vertex_array.setPrimitiveType(sf::Triangles);
    }

    void setSize(double size)
    {
        this->size = size;
    }

    void setThickness(double thickness)
    {
        this->thickness = thickness;
    }

    void setColor(sf::Color color)
    {
        this->color = color;
    }

    void update(const std::vector<Eigen::Vector2d> positions)const
    {
        vertex_array.clear();
        for (const auto& position: positions) {
            add_marker(vertex_array, position, size, color);
            // TODO: Use thickness
        }
    }

private:

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        states.transform *= getTransform();
        target.draw(vertex_array, states);
    }

    double size;
    double thickness;
    sf::Color color;

    mutable sf::VertexArray vertex_array;
};

class Target: public sf::Drawable {
public:
    Eigen::Vector2d position;
    mutable Marker marker;

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        marker.setPosition(position.x(), position.y());
        target.draw(marker, states);
    }
};
*/

}

#endif