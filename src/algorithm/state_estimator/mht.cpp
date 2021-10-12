
#include "algorithm/state_estimator.h"


StateEstimatorMht::StateEstimatorMht(
        const MotionModel& motion_model,
        const MeasurementModel& range_model,
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
    addStep(std::bind(&StateEstimatorMht::prune, this), "prune");
}

Pose StateEstimatorMht::getStateEstimate()const
{
    if (components.empty()) return Pose();

    double L_best = -INFINITY;
    size_t i_best = 0;
    for (size_t i = 0; i < components.size(); i++) {
        double L = components[i].log_likelihood;
        if (L > L_best) {
            L_best = L;
            i_best = i;
        }
    }
    return components[i_best].pose;
}

void StateEstimatorMht::resetEstimate(const Pose& pose)
{
    size_t num_components = 10; // TODO: Make this configurable

    components.clear();
    components.resize(num_components);
    for (auto& component: components) {
        component.log_likelihood = 0;
        sampleInitialComponent(pose, component.pose, component.covariance);
    }
}

bool StateEstimatorMht::predict()
{
    for (auto& component: components) {
        auto linear_model = motion_model.linearise(component.pose, *twistEstimate);
        ekfPredict(component.pose.state(), component.covariance, linear_model);
    }
    return true;
}

bool StateEstimatorMht::feature_detection()
{
    // Reset match result, so it doesn't render old information
    for (auto& component: components) {
        component.match_result.matches.clear();
        component.match_result.known_features = nullptr;
    }
    feature_detector.findFeatures(*ranges, features);
    for (auto& component: components) {
        component.match_result.observed_features = &features;
    }
    return true;
}

bool StateEstimatorMht::feature_matching()
{
    for (auto& component: components) {
        terrain->queryKnownFeatures(
            component.pose,
            component.match_result.known_features,
            component.match_result.indicators);
        feature_matcher.findMatches(component.pose, component.match_result);
    }
    return true;
}

bool StateEstimatorMht::update()
{
    for (auto& component: components) {
        std::vector<LinearModelUpdate<3, 2>> linear_models(component.match_result.matches.size());
        std::vector<Point> y(component.match_result.matches.size());

        for (size_t i = 0; i < component.match_result.matches.size(); i++) {
            const Point& y_prior = component.match_result.known_features->points[component.match_result.matches[i].index_known];
            const Point& y = component.match_result.observed_features->points[component.match_result.matches[i].index_observed];

            linear_models[i] = feature_model.linearise(component.pose, y_prior, y);
        }
        ekfUpdateMultipleWithLikelihood(component.pose.state(), component.covariance, component.log_likelihood, linear_models);
    }

    // Re-normalize so max log_likelihood is zero
    double L_max = std::accumulate(components.begin(), components.end(), (double)(-INFINITY), [](double value, const Component& comp) {
        return comp.log_likelihood > value ? comp.log_likelihood : value;
    });

    for (auto& component: components) {
        component.log_likelihood -= L_max;
    }

    for (size_t i = 0; i < components.size();) {
        if (components[i].log_likelihood < -100 && components.size() != 1) {
            components[i] = components.back();
            components.pop_back();
        } else {
            i++;
        }
    }

    return true;
}

bool StateEstimatorMht::prune()
{
    double epsilon1 = 0.1;
    double epsilon2 = 1;
    return true;
}

void StateEstimatorMht::sampleInitialComponent(const Pose& mean, Pose& pose, Eigen::Matrix3d& cov)const
{
    Eigen::Matrix3d sample_cov = Eigen::Vector3d(0.4, 0.4, 0.08).asDiagonal();
    Eigen::Matrix3d prior_cov = Eigen::Vector3d(0.4, 0.4, 0.08).asDiagonal();

    pose.state() = sampleGaussian(mean.state(), sample_cov);
    cov = prior_cov;
}
