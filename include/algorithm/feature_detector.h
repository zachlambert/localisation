#ifndef ALGORITHM_FEATURE_DETECTOR
#define ALGORITHM_FEATURE_DETECTOR

#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "state/robot.h"
#include "maths/measurement_model.h"
#include "maths/feature_model.h"


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

    struct Config
    {
        double false_negative_p;
        double false_positive_rate;
    };
    void setConfig(const Config& config) { this->config = config; }

    void findFeatures(
        const PointCloud& measurements,
        PointCloud& features)const
    {
        features.points.clear();

        const PointCloud* known_features;
        std::vector<bool> indicators;
        terrain.queryKnownFeatures(robot.pose, known_features, indicators);

        if (known_features->points.empty()) return;
        const size_t DESCRIPTOR_SIZE = known_features->points[0].descriptor.size();

        for (size_t i = 0; i < known_features->points.size(); i++) {
            if (!indicators[i]) continue;

            // Random probability of missing a feature (false negative)
            if (sampleUniform(0, 1) < config.false_negative_p) {
                continue;
            }

            // Sample from feature model to add random error
            features.points.push_back(feature_model.sample(robot.pose, known_features->points[i]));
        }

        // Random number of false positives, using poission distribution
        int num_false_positives = samplePoisson(config.false_positive_rate);
        for (size_t i = 0; i < num_false_positives; i++) {
            // Pick measurement randomly
            const Point& measurement = measurements.points[rand() % measurements.points.size()];
            // Random landmark descriptor
            Eigen::VectorXd descriptor = randomDescriptor(DESCRIPTOR_SIZE);
            Point feature;
            feature.setPolar(measurement.range(), measurement.angle());
            feature.descriptor = descriptor;
            features.points.push_back(feature);
        }
    }

private:
    Config config;
    const Terrain& terrain;
    const Robot& robot;
    const FeatureModel& feature_model;
};


#endif
