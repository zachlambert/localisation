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

    void addElementLandmarks(const Element& element, size_t descriptor_size);
    double queryIntersection(const Pose& pose, double angle)const;

    void queryKnownFeatures(const Pose& pose, const PointCloud*& known_features, std::vector<bool>& indicators)const
    {
        static constexpr double intersection_allowance = 0.1;

        known_features = &landmarks;
        indicators.resize(known_features->points.size());

        for (size_t i = 0; i < known_features->points.size(); i++) {
            const Point& feature = known_features->points[i];
            double known_range = queryIntersection(pose, feature.angle(pose));

            if (feature.range(pose) < known_range + intersection_allowance) {
                indicators[i] = true;
            } else {
                indicators[i] = false;
            }
        }
    }
};


void createTerrain(Terrain& terrain);

#endif
