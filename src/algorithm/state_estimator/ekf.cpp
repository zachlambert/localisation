
#include "algorithm/state_estimator.h"


StateEstimatorEkf::StateEstimatorEkf(
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
    auto f = motion_model.linearise(estimate.pose, *twistEstimate);
    ekfPredict(estimate, f);
    return true;
}

bool StateEstimatorEkf::feature_detection()
{
    feature_detector.findFeatures(*ranges, features);
    return true;
}

bool StateEstimatorEkf::feature_matching()
{
    terrain->getObservableLandmarks(estimate.pose, features_prior);
    feature_matcher.getCorrespondances(correspondances, features, features_prior);
    return true;
}

bool StateEstimatorEkf::update()
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
