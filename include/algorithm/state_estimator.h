#ifndef ALGORITHM_STATE_ESTIMATOR_H
#define ALGORITHM_STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <iostream>

#include "maths/geometry.h"
#include "maths/motion_model.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "utils/multistep.h"
#include "maths/measurement_model.h"
#include "algorithm/feature_detector.h"


class StateEstimator: public Multistep {
public:
    StateEstimator(
            const MotionModel& motion_model,
            const RangeModel& range_model,
            const FeatureModel& feature_model,
            const FeatureDetector& feature_detector,
            const FeatureMatcher& feature_matcher):
        motion_model(motion_model),
        range_model(range_model),
        feature_model(feature_model),
        feature_detector(feature_detector),
        feature_matcher(feature_matcher)
    {
    }

    // General function for getting a state estimate.
    // May store the state estimate explicitly, or may calculate
    // using a parameterisation (eg: particles).
    virtual Pose getStateEstimate()const = 0;

    virtual void resetEstimate(const Pose& pose) = 0;

    // Provide the data for a new predict and update step.
    // Needs the following information in general.
    // - Control data (odometry)
    // - Observations (ranges)
    // - Known map (terrain) [May or may not be used]
    void start(Velocity* twistEstimate, const PointCloud* ranges, const Terrain* terrain)
    {
        this->twistEstimate = twistEstimate;
        this->ranges = ranges;
        this->terrain = terrain;
        Multistep::start();
    }

protected:

    // Models
    const MotionModel& motion_model;
    const RangeModel& range_model;
    const FeatureModel& feature_model;
    const FeatureDetector& feature_detector;
    const FeatureMatcher& feature_matcher;

    // Inputs
    const Velocity* twistEstimate;
    const PointCloud* ranges = nullptr;
    const Terrain* terrain = nullptr;
};


class StateEstimatorEkf: public StateEstimator {
public:
    struct Estimate {
        Pose pose;
        Eigen::Matrix3d covariance;
    };

    Estimate estimate;
    PointCloud features;
    PointCloud features_prior;
    std::vector<Correspondance> correspondances;

    StateEstimatorEkf(
            const MotionModel& motion_model,
            const RangeModel& range_model,
            const FeatureModel& feature_model,
            const FeatureDetector& feature_detector,
            const FeatureMatcher& feature_matcher);

    virtual Pose getStateEstimate()const;
    virtual void resetEstimate(const Pose& pose);

private:
    bool predict();
    bool feature_detection();
    bool feature_matching();
    bool update();

    int step_number = 0;
};


class StateEstimatorMht: public StateEstimator {
public:
    struct Estimate {
        struct Component {
            Pose pose;
            Eigen::Matrix3d covariance;
            double probability;
        };
        std::vector<Component> components;
    };

    Estimate estimate;
    PointCloud features;

    struct FeatureMatchResult {
        PointCloud features_prior;
        std::vector<Correspondance> correspondances;
    };
    std::vector<FeatureMatchResult> component_feature_match_results;

    StateEstimatorMht(
            const MotionModel& motion_model,
            const RangeModel& range_model,
            const FeatureModel& feature_model,
            const FeatureDetector& feature_detector,
            const FeatureMatcher& feature_matcher);

    virtual Pose getStateEstimate()const;
    virtual void resetEstimate(const Pose& pose);

private:
    bool predict();
    bool feature_detection();
    bool feature_matching();
    bool update();

    void sampleInitialComponent(Pose& pose, Eigen::Matrix3d& cov)const;

    int step_number = 0;
};


/* TODO
class StateEstimatorGrid: public StateEstimator {
public:
    GaussianStateEstimate estimate;
     // Public for rendering
    PointCloud features;
    PointCloud features_prior;
    std::vector<Correspondance> correspondances;

    StateEstimatorGrid(
            const MotionModel& motion_model,
            const RangeModel& range_model,
            const FeatureModel& feature_model,
            const FeatureDetector& feature_detector,
            const FeatureMatcher& feature_matcher);

    virtual Pose getStateEstimate()const;
    virtual void resetEstimate(const Pose& pose);

private:
    bool predict();
    bool feature_detection();
    bool feature_matching();
    bool update();

private:
    int step_number = 0;
};
*/

#endif
