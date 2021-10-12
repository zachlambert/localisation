
#include "algorithm/state_estimator.h"


StateEstimatorEkf::StateEstimatorEkf(
        const MotionModel& motion_model,
        const MeasurementModel& measurement_model,
        const FeatureModel& feature_model,
        const FeatureDetector& feature_detector,
        const FeatureMatcher& feature_matcher):
    StateEstimator(
        motion_model,
        measurement_model,
        feature_model,
        feature_detector,
        feature_matcher)
{
    addStep(std::bind(&StateEstimatorEkf::predict, this), "predict");
    addStep(std::bind(&StateEstimatorEkf::feature_detection, this), "feature detection");
    addStep(std::bind(&StateEstimatorEkf::feature_matching, this), "feature matching");
    addStep(std::bind(&StateEstimatorEkf::update, this), "update");
}

Pose StateEstimatorEkf::getStateEstimate()const
{
    return estimate.pose;
}

void StateEstimatorEkf::resetEstimate(const Pose& pose)
{
    estimate.pose = pose;
    estimate.covariance.setZero();
}

bool StateEstimatorEkf::predict()
{
    auto linear_model = motion_model.linearise(estimate.pose, *twistEstimate);
    ekfPredict(estimate.pose.state(), estimate.covariance, linear_model);
    return true;
}

bool StateEstimatorEkf::feature_detection()
{
    // Reset match result, so it doesn't render old information
    match_result.matches.clear();
    match_result.known_features = nullptr;

    feature_detector.findFeatures(*ranges, features);
    match_result.observed_features = &features;
    return true;
}

bool StateEstimatorEkf::feature_matching()
{
    terrain->queryKnownFeatures(estimate.pose, match_result.known_features, match_result.indicators);
    feature_matcher.findMatches(estimate.pose, match_result);
    return true;
}

bool StateEstimatorEkf::update()
{
    std::vector<LinearModelUpdate<3, Eigen::Dynamic>> linear_models(match_result.matches.size());
    std::vector<Point> y(match_result.matches.size());

    for (size_t i = 0; i < match_result.matches.size(); i++) {
        const Point& y_prior = match_result.known_features->points[match_result.matches[i].index_known];
        const Point& y = match_result.observed_features->points[match_result.matches[i].index_observed];

        linear_models[i] = feature_model.linearise(estimate.pose, y_prior, y);
    }
    ekfUpdateMultiple(estimate.pose.state(), estimate.covariance, linear_models);
    return true;
}
