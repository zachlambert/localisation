#ifndef TERRAIN_H
#define TERRAIN_H

/*  The Terrain class allows querying ray intersections with the environment
 *  to simulate sensors.
 *  Specific sensors use it as part of their measurement model.
 *
 *  Not used for probabilistic representation of a map.
 *
 *  Terrain can't be changed after initialising.
 */

#include <vector>
#include <Eigen/Core>
#include <SFML/Graphics.hpp>

#include "geometry.h"


class Terrain {
public:
    struct Element {
        Eigen::Vector2d pos;
        std::vector<Eigen::Vector2d> vertices;
        Element(): pos(0,0), vertices() {}
        void add_vertex(double x, double y) {
            vertices.push_back(Eigen::Vector2d(x, y));
        }
    };

    Terrain(sf::Color color): color(color) {}
    void initialise();
    void add_element(const Element &element)
    {
        elements.push_back(element);
    }
    double query_intersection(const Pose& pose, double angle)const;

    Pose pose;
    struct {
        sf::VertexArray terrain;
    } to_render;

private:
    sf::Color color;
    std::vector<Element> elements;
};


#endif
