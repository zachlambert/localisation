
#include "algorithm/state_estimator.h"


StateEstimatorMht::StateEstimatorMht(
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
    addStep(std::bind(&StateEstimatorMht::predict, this), "predict");
    addStep(std::bind(&StateEstimatorMht::feature_detection, this), "feature detection");
    addStep(std::bind(&StateEstimatorMht::feature_matching, this), "feature matching");
    addStep(std::bind(&StateEstimatorMht::update, this), "update");
}

Pose StateEstimatorMht::getStateEstimate()const
{
    if (estimate.components.empty()) return Pose();

    double p_best = 0;
    size_t i_best = 0;
    for (size_t i = 0; i < estimate.components.size(); i++) {
        double p = estimate.components[i].probability;
        if (p > p_best) {
            p_best = p;
            i_best = i;
        }
    }
    return estimate.components[i_best].pose;
}

void StateEstimatorMht::resetEstimate(const Pose& pose)
{
    size_t num_components = 10; // TODO: Make this configurable

    estimate.components.clear();
    estimate.components.resize(num_components);
    for (size_t i = 0; i < estimate.components.size(); i++) {
        Estimate::Component& component = estimate.components[i];
        component.probability = (double)1 / num_components;
        sampleInitialComponent(component.pose, component.covariance);
    }

    component_feature_match_results.resize(num_components);
}

bool StateEstimatorMht::predict()
{
    for (size_t i = 0; i < estimate.components.size(); i++) {
        Estimate::Component& component = estimate.components[i];

        auto linear_model = motion_model.linearise(component.pose, *twistEstimate);
        ekfPredict(component.pose.state(), component.covariance, linear_model);
    }
    return true;
}

bool StateEstimatorMht::feature_detection()
{
    feature_detector.findFeatures(*ranges, features);
    return true;
}

bool StateEstimatorMht::feature_matching()
{
    for (size_t i = 0; i < estimate.components.size(); i++) {
        Estimate::Component& component = estimate.components[i];

        terrain->getObservableLandmarks(component.pose, component_feature_match_results[i].features_prior);
        feature_matcher.getCorrespondances(
            component_feature_match_results[i].correspondances,
            features,
            component_feature_match_results[i].features_prior);
    }
    return true;
}

bool StateEstimatorMht::update()
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

void sampleInitialComponent(Pose& pose, Eigen::Matrix3d& cov)const
{

}
