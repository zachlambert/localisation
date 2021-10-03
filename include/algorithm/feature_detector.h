#ifndef ALGORITHM_FEATURE_DETECTOR
#define ALGORITHM_FEATURE_DETECTOR

#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "state/robot.h"
#include "maths/measurement_model.h"


class FeatureDetector {
public:
    virtual void findFeatures(const PointCloud& ranges, PointCloud& features)const = 0;
};

class FeatureDetectorFake: public FeatureDetector {
public:
    FeatureDetectorFake(const Terrain& terrain, const Robot& robot, const FeatureModel& feature_model):
        terrain(terrain),
        robot(robot),
        feature_model(feature_model)
    {}
    virtual void findFeatures(const PointCloud& ranges, PointCloud& features)const
    {
        feature_model.sampleFakeFeatures(features, robot.pose, terrain);
    }
private:
    const Terrain& terrain;
    const Robot& robot;
    const FeatureModel& feature_model;
};

class FeatureDetectorCorners: public FeatureDetector {
public:
    virtual void findFeatures(const PointCloud& ranges, PointCloud& features)const
    {
        // TODO
    }
};

#endif
