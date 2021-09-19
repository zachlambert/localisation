#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H

#include <iostream>
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

void add_circle_mesh(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double radius,
    sf::Color color,
    size_t n=20);


class Marker: public sf::Drawable, public sf::Transformable {
public:
    Marker():
        size(0.1),
        thickness(0.02),
        color(sf::Color::Red),
        dirty(true)
    {
        vertex_array.setPrimitiveType(sf::Triangles);
    }

    void setSize(double size)
    {
        this->size = size;
        dirty = true;
    }

    void setThickness(double thickness)
    {
        this->thickness = thickness;
        dirty = true;
    }

    void setColor(sf::Color color)
    {
        this->color = color;
        dirty = true;
    }

private:
    void update_vertices()const
    {
        if (!dirty) return;

        vertex_array.clear();
        add_marker(vertex_array, Eigen::Vector2d(0,0), size, color);
        for (size_t i = 0; i < vertex_array.getVertexCount(); i++) {
            std::cout << vertex_array[i].position.x << ", "
                      << vertex_array[i].position.y << std::endl;
        }
        std::cout << "here" << std::endl;
        // TODO: Use thickness

        dirty = false;
    }

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        update_vertices();
        states.transform *= getTransform();
        target.draw(vertex_array, states);
    }

    double size;
    double thickness;
    sf::Color color;

    mutable bool dirty;
    mutable sf::VertexArray vertex_array;
};


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

#endif
