
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
    feature_detector.findFeatures(*ranges, features);
    match_result.observed_features = &features;
    return true;
}

bool StateEstimatorEkf::feature_matching()
{
    terrain->setFeatureIdentifiers(estimate.pose, match_result);
    terrain->getObservableLandmarks(estimate.pose, features_prior);
    feature_matcher.getCorrespondances(correspondances, features, features_prior);
    return true;
}

bool StateEstimatorEkf::update()
{
    std::vector<LinearModelUpdate<3, Eigen::Dynamic>> linear_models(correspondances.size());
    std::vector<Point> y(correspondances.size());

    for (size_t i = 0; i < correspondances.size(); i++) {
        const Point& y_prior = features_prior.points[correspondances[i].index_prior];
        const Point& y = features.points[correspondances[i].index_new];

        linear_models[i] = feature_model.linearise(estimate.pose, y_prior, y);
    }
    ekfUpdateMultiple(estimate.pose.state(), estimate.covariance, linear_models);
    return true;
}
