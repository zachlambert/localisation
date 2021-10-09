
#include "algorithm/state_estimator.h"


StateEstimatorHmm::StateEstimatorHmm(
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
    addStep(std::bind(&StateEstimatorHmm::predict, this), "predict");
    addStep(std::bind(&StateEstimatorHmm::feature_detection, this), "feature detection");
    addStep(std::bind(&StateEstimatorHmm::feature_matching, this), "feature matching");
    addStep(std::bind(&StateEstimatorHmm::update, this), "update");
}

Pose StateEstimatorHmm::getStateEstimate()const
{
    return estimate.pose;
}

void StateEstimatorEKF::resetEstimate(const Pose& pose)
{
    estimate.pose = pose;
    estimate.covariance.setZero();
}

bool StateEstimatorEKF::predict()
{
    auto f = motion_model.linearise(estimate.pose, *twistEstimate);
    ekfPredict(estimate, f);
    return true;
}

bool StateEstimatorEKF::feature_detection()
{
    feature_detector.findFeatures(*ranges, features);
    return true;
}

bool StateEstimatorEKF::feature_matching()
{
    terrain->getObservableLandmarks(estimate.pose, features_prior);
    feature_matcher.getCorrespondances(correspondances, features, features_prior);
    return true;
}

bool StateEstimatorEKF::update()
{
    std::vector<LinearisedObservationEquation> g(correspondances.size());
    std::vector<Point> y(correspondances.size());
    for (size_t i = 0; i < correspondances.size(); i++) {
        g[i] = feature_model.linearise(estimate.pose, features_prior.points[correspondances[i].index_prior]);
        y[i] = features.points[correspondances[i].index_new];
    }
    ekfUpdateMultiple(estimate, y, g);
    return true;
}
