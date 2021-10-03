#ifndef ALGORITHM_STATE_ESTIMATOR_H
#define ALGORITHM_STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "maths/geometry.h"
#include "maths/motion_model.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"
#include "utils/step.h"
#include "maths/measurement_model.h"
#include "algorithm/feature_detector.h"


class StateEstimator: public Step<StateEstimator> {
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
        addStep(&StateEstimator::predict);
        addStep(&StateEstimator::update);
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
        Step::start();
    }

protected:
    virtual bool predict() = 0;
    virtual bool update() = 0;

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


class StateEstimatorEKF: public StateEstimator {
public:
    StateEstimateGaussian x;

    StateEstimatorEKF(
            const MotionModel& motion_model,
            const RangeModel& range_model,
            const FeatureModel& feature_model,
            const FeatureDetector& feature_detector,
            const FeatureMatcher& feature_matcher):
        StateEstimator(
            motion_model,
            range_model,
            feature_model,
            feature_detector,
            feature_matcher)
    {}

    virtual Pose getStateEstimate()const
    {
        return x.pose;
    }

    virtual void resetEstimate(const Pose& pose)
    {
        x.pose = pose;
        x.covariance.setZero();
    }

protected:
    virtual bool predict()
    {
        x = motion_model.getGaussian(x, *twistEstimate);
        return true;
    }

    virtual bool update()
    {
        if (!features_found) {
            feature_detector.findFeatures(*ranges, features);
            features_found = true;
            return false;
        }

        features_found = false;
        return true;
    }
private:
    bool features_found = false;
    PointCloud features;
};

#endif
