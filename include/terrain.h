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


class Terrain: public sf::Drawable {
public:
    struct Element {
        Eigen::Vector2d pos;
        std::vector<Eigen::Vector2d> vertices;
        Element(): pos(0,0), vertices() {}
        void addVertex(double x, double y) {
            vertices.push_back(Eigen::Vector2d(x, y));
        }
    };
    struct Landmark {
        Eigen::Vector2d pos;
        // May add extra information later
        Landmark(): pos() {}
        Landmark(double x, double y): pos(x, y) {}
    };

    Terrain();
    void addElement(const Element &element);
    void addLandmark(const Landmark &landmark);

    void setTerrainColor(sf::Color color);
    void setLandmarkColor(sf::Color color);
    void setLandmarkSize(double size);

    double queryIntersection(const Pose& pose, double angle)const;

private:
    void updateVertices()const;
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    // Render information
    sf::Color terrain_color;
    sf::Color landmark_color;
    double landmark_size;
    mutable sf::VertexArray vertex_array;
    mutable bool dirty;

    std::vector<Element> elements;
    std::vector<Landmark> landmarks;
};


void createTerrain(Terrain& terrain);

#endif
