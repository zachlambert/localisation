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
#include "point_cloud.h"


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

    Terrain();
    void addElement(const Element &element, bool add_landmarks=true);

    void setTerrainColor(sf::Color color);
    void setLandmarkColor(sf::Color color);
    void setLandmarkSize(double size);

    double queryIntersection(const Pose& pose, double angle)const;
    void getObservableLandmarks(const Pose& pose, PointCloud& landmarks)const;

    void updateVertices();

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states)const;

    // Render information
    sf::Color terrain_color;
    sf::Color landmark_color;
    double landmark_size;
    sf::VertexArray vertex_array;

    std::vector<Element> elements;
    PointCloud landmarks;
};


void createTerrain(Terrain& terrain);

#endif
