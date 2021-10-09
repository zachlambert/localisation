#ifndef ALGORITHM_MAP_H
#define ALGORITHM_MAP_H

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"


class Map {
public:
    // Return the maximum likelihood intersection distance for a given
    // pose and viewing angle
    virtual double queryIntersection(const Pose& pose, double angle)const = 0;

    // Returns true if the given position intersects the environment.
    // TODO Add when necessary
    virtual bool queryOccupancy(const Eigen::Vector2d& pos)const { return false; }

    // Provide a pointer to a list of known features, and if applicable,
    // set an indicator list, specifying which features could feasibly be
    // observed for a given pose.
    virtual void queryKnownFeatures(const Pose& pose, const PointCloud*& known_features, std::vector<bool>& indicators)const = 0;
};


class TerrainMap: public Map {
public:
    TerrainMap(const Terrain& terrain):
        terrain(terrain)
    {}

    virtual double queryIntersection(const Pose& pose, double angle)const
    {
        return terrain.queryIntersection(pose, angle);
    }

    virtual void queryKnownFeatures(const Pose& pose, const PointCloud*& known_features, std::vector<bool>& indicators)const
    {
        static constexpr double intersection_allowance = 0.1;

        known_features = &terrain.landmarks;
        indicators.resize(known_features->points.size());

        for (size_t i = 0; i < known_features->points.size(); i++) {
            const Point& feature = known_features->points[i];
            double known_range = queryIntersection(pose, feature.angle);

            if (feature.range < known_range + intersection_allowance) {
                indicators[i] = true;
            } else {
                indicators[i] = false;
            }
        }
    }

private:
    const Terrain& terrain;
};


// TODO
class DynamicMap: public Map {

};

#endif
