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

#include "maths/geometry.h"
#include "maths/point_cloud.h"


class Terrain {
public:
    struct Element {
        Eigen::Vector2d pos;
        std::vector<Eigen::Vector2d> vertices;
        Element(): pos(0,0), vertices() {}
        void addVertex(double x, double y) {
            vertices.push_back(Eigen::Vector2d(x, y));
        }
    };

    std::vector<Element> elements;
    PointCloud landmarks;

    void addElementLandmarks(const Element& element);
    double queryIntersection(const Pose& pose, double angle)const;
    void getObservableLandmarks(const Pose& pose, PointCloud& landmarks)const;

    Eigen::VectorXd randomLandmarkDescriptor()const;
};


void createTerrain(Terrain& terrain);

#endif
