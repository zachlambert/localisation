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


class StateEstimatorEKF: public StateEstimator {
public:
    StateEstimateGaussian x;
     // Public for rendering
    PointCloud features;
    PointCloud features_prior;
    std::vector<Correspondance> correspondances;

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
    {
        addStep(std::bind(&StateEstimatorEKF::predict, this), "predict");
        addStep(std::bind(&StateEstimatorEKF::feature_detection, this), "feature detection");
        addStep(std::bind(&StateEstimatorEKF::feature_matching, this), "feature matching");
        addStep(std::bind(&StateEstimatorEKF::update, this), "update");
    }

    virtual Pose getStateEstimate()const
    {
        return x.pose;
    }

    virtual void resetEstimate(const Pose& pose)
    {
        x.pose = pose;
        x.covariance.setZero();
    }

private:
    bool predict()
    {
        x = motion_model.getGaussian(x, *twistEstimate);
        // TODO
        // Replace with:
        // motion_model.getLinearModel()
        // kalman_filter.predict(...)
        return true;
    }

    bool feature_detection()
    {
        correspondances.clear();
        feature_detector.findFeatures(*ranges, features);
        return true;
    }

    bool feature_matching()
    {
        terrain->getObservableLandmarks(x.pose, features_prior);
        feature_matcher.getCorrespondances(correspondances, features, features_prior);
        return true;
    }

    bool update()
    {
        // TODO
        // feature_model.getLinearModel()
        // kalman_filter.update_nonlinear(...)
        return true;
    }

private:
    int step_number = 0;
};

#endif
