#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

// Models how features are observed.
// - Adds gaussian noise to range and angle.
// - Adds gaussian noise to descriptor.
// - Probability of a false negative (missing a valid feature)
// - Probability of a false positive (adding an invalid feature)

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "maths/probability.h"
#include "state/terrain.h"


class RangeModel {
public:
    struct Config {
        double prior_hit;
        double prior_invalid; // Get max range
        double prior_short; // Hits nearby dynamic object
        double prior_random; // Uniform over [0, max_range]

        double max_range;
        double short_scale; // Exponential distribution
        double hit_var; // Variance about hit range
    };
    void setConfig(const Config& config) { this->config = config; }

    double evaluateProbability(double range, double angle, const Pose& pose, const Terrain& terrain)
    {
        double range_hit = terrain.queryIntersection(pose, angle);

        double prob_inf = config.prior_invalid;
        if (range_hit > config.max_range) {
            prob_inf += config.prior_hit;
        }

        if (std::isinf(range)) {
            // Prob_inf * uniform over all measurements
            return prob_inf * (1 / config.max_range);
        }

        // Otherwise (1-prob_inf) * p(y | range_hit)

        double p_hit = evaluateGaussian(range_hit, range, config.hit_var);
        double p_short = evaluateExponential(range, config.short_scale);
        double p_random = evaluateUniform(0, config.max_range);

        double p_y =
            config.prior_hit * p_hit +
            config.prior_short * p_short +
            config.prior_random * p_random;
        return (1 - prob_inf) * p_y;
    }

    double sample(double angle, const Pose& pose, const Terrain& terrain)
    {
        double range_hit = terrain.queryIntersection(pose, angle);

        double prob_inf = config.prior_invalid;
        if (range_hit > config.max_range) {
            prob_inf += config.prior_hit;
        }

        if (sampleUniform(0, 1) < prob_inf) {
            return INFINITY;
        }

        double u = sampleUniform(0, config.prior_hit + config.prior_short + config.prior_random);
        if (u < config.prior_hit) {
            return sampleGaussian(range_hit, config.hit_var);
        } else if (u < config.prior_hit + config.prior_short) {
            return sampleExponential(config.short_scale);
        } else {
            return sampleUniform(0, config.max_range);
        }
    }

    struct RangeEstimate {
        double range;
        double variance;
    };
    RangeEstimate getRangeEstimate(double angle, const Pose& pose, const Terrain& terrain)
    {
        RangeEstimate range_estimate;
        range_estimate.range = terrain.queryIntersection(pose, angle);
        range_estimate.variance = config.hit_var;
        return range_estimate;
    }

private:
    Config config;
};

class FeatureModel {
public:
    FeatureModel(const RangeModel& range_model):
        range_model(range_model)
    {}

    struct Config {
        double range_var;
        double angle_var;
        double descriptor_var;
        double false_negative_p;
        double false_positive_rate;

        double correspondance_p_threshold;
    };

    void setConfig(const Config& config) { this->config = config; }

    void sampleFeatures(PointCloud& features, const Pose& pose, const Terrain& terrain)
    {
        terrain.getObservableLandmarks(pose, features);

        for (size_t i = 0; i < features.points.size(); i++) {
            // Random probability of missing a feature (false negative)
            if (sampleUniform(0, 1) < config.false_negative_p) {
                features.points[i] = features.points.back();
                features.points.pop_back();
                features.descriptors[i] = features.descriptors.back();
                features.descriptors.pop_back();
                continue;
            }
            // Otherwise, add gaussian noise
            features.points[i].setPolar(
                features.points[i].dist() + sampleGaussian(0, config.range_var),
                features.points[i].angle() + sampleGaussian(0, config.angle_var));
            for (size_t j = 0; j < features.descriptors[i].size(); j++) {
                double& value = features.descriptors[i](j);
                value += sampleGaussian(0, config.descriptor_var); 
                if (value < 0) value = 0;
            }
            // Renormalise so max value = 1
            features.descriptors[i] /= features.descriptors[i].sum();
        }

        // Random number of false positives, using poission distribution
        int num_false_positives = samplePoisson(config.false_positive_rate);
        for (size_t i = 0; i < num_false_positives; i++) {
            // Pick angle randomly
            double angle = sampleUniform(-M_PI, M_PI);
            // Ray cast distance
            double dist = terrain.queryIntersection(pose, angle);
            // Random landmark descriptor
            Eigen::VectorXd descriptor = terrain.randomLandmarkDescriptor();
            features.points.push_back(Point(dist, angle));
            features.descriptors.push_back(descriptor);
        }
    }

    double evaluateProbability(PointCloud& features, const Pose& pose, const Terrain& terrain)
    {

    }

    struct LinearModel {
        Eigen::VectorXd innovation;
        Eigen::MatrixXd C;
    };
    LinearModel getLinearModel(const PointCloud& y, const Pose& pose, const Terrain& terrain)
    {
        PointCloud y_predicted;
        terrain.getObservableLandmarks(pose, y_predicted);

        LinearModel linear_model;
        linear_model.y_mean = true_features.getState();
    }

    double evaluateProbabilitySingle(const PointCloud& y, const PointCloud& y_mean, size_t index)
    {

    }

    struct Correspondance {
        size_t index_prior; // Known map or previous observation
        size_t index_new; // New observation
        double p;
        Correspondance(size_t index_prior, size_t index_new, double p):
            index_prior(index_prior), index_new(index_new), p(p)
        {}
    };
    void getCorrespondances(std::vector<Correspondance>& c, const PointCloud& y_new, const PointCloud& y_prior)
    {
        for (size_t i = 0; i < y_new.points.size(); i++) {
            double p_best = 0;
            size_t j_best = 0;
            for (size_t j = 0; j < y_prior.points.size(); j++) {
                double p_j = evaluateProbabilitySingle(y_new.points[i], y_prior.points[j]);
            }
            if (p_best > config.correspondance_p_threshold) {
                c.push_back(Correspondance(j_best, i, p_best));
            }
        }
    }

private:
    Config config;
    const RangeModel& range_model;
};

#endif
