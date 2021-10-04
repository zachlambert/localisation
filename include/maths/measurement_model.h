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
#include "algorithm/kalman_filter.h"
#include <iostream>


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

    double evaluateProbability(double range, double angle, const Pose& pose, const Terrain& terrain)const
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

    double sample(double angle, const Pose& pose, const Terrain& terrain)const
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
    RangeEstimate getRangeEstimate(double angle, const Pose& pose, const Terrain& terrain)const
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
    struct Config {
        double range_var;
        double angle_var;
        double descriptor_var;
        double false_negative_p;
        double false_positive_rate;
    };

    void setConfig(const Config& config) { this->config = config; }

    // for FeatureDetectorFake only
    void sampleFakeFeatures(PointCloud& features, const Pose& pose, const Terrain& terrain)const
    {
        terrain.getObservableLandmarks(pose, features);

        for (size_t i = 0; i < features.points.size(); i++) {
            // Random probability of missing a feature (false negative)
            if (sampleUniform(0, 1) < config.false_negative_p) {
                features.points[i] = features.points.back();
                features.points.pop_back();
                continue;
            }

            // Otherwise, add gaussian noise

            features.points[i].range += sampleGaussian(0, config.range_var);
            features.points[i].angle += sampleGaussian(0, config.angle_var);

            for (size_t j = 0; j < features.points[i].descriptor.size(); j++) {
                double& value = features.points[i].descriptor(j);
                value += sampleGaussian(0, config.descriptor_var); 
                if (value < 0) value = 0;
            }
            // Renormalise so max value = 1
            features.points[i].descriptor /= features.points[i].descriptor.sum();
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
            features.points.push_back(Point(dist, angle, terrain.randomLandmarkDescriptor()));
        }
    }

    double evaluateProbabilitySingle(const Point& yi, const Point& yi_mean)const
    {
        double p = 1;
        p *= evaluateGaussian(yi.range, yi_mean.range, config.range_var);
        double angle_dif = yi.angle - yi_mean.angle;
        normaliseAngle(angle_dif);
        p *= evaluateGaussian(0, angle_dif, config.angle_var);
        for (size_t j = 0; j < yi_mean.descriptor.size(); j++) {
            p *= evaluateGaussian(yi.descriptor(j), yi_mean.descriptor(j), config.descriptor_var);
        }
        return p;
    }

    double evaluateProbability(const PointCloud& y, const PointCloud& y_prior, const std::vector<Correspondance>& correspondances)const
    {
        double p = 1;
        for (size_t i = 0; i < correspondances.size(); i++) {
            p *= evaluateProbabilitySingle(
                y.points[correspondances[i].index_new],
                y_prior.points[correspondances[i].index_prior]);
        }
        return p;
    }

    LinearisedObservationEquation linearise(const Pose& x_prior, const Point& y_prior)const
    {
        LinearisedObservationEquation g;
        size_t Ny = y_prior.state().size();

        g.y_prior = y_prior;
        g.C.resize(Ny, 3);
        g.R.resize(Ny, Ny);
        g.R.setZero();

        Eigen::VectorXd dir = Eigen::Rotation2Dd(x_prior.orientation()) * y_prior.pos();
        dir.normalize();

        g.C.setZero();
        g.C.block(0, 0, 1, 2) = - dir.transpose();
        g.C.block(1, 0, 1, 2) = - (crossProductMatrix(1) * dir).transpose() / y_prior.range;
        g.C(1, 2) = -1;

        g.R(0, 0) = config.range_var;
        g.R(1, 1) = config.angle_var;
        g.R.block(2, 2, Ny-2, Ny-2).setIdentity();
        g.R.block(2, 2, Ny-2, Ny-2) *= config.descriptor_var;

        return g;
    }

private:
    Config config;
};

class FeatureMatcher {
public:
    FeatureMatcher(const FeatureModel& feature_model):
        feature_model(feature_model)
    {}

    struct Config {
        bool use_feature_model; // Otherwise just nearest neighbour search of descriptors
        double correspondance_p_threshold;
    };
    void setConfig(const Config& config) { this->config = config; }

    void getCorrespondances(std::vector<Correspondance>& c, const PointCloud& y_new, const PointCloud& y_prior)const
    {
        c.clear();
        for (size_t i = 0; i < y_new.points.size(); i++) {
            double score_best = 0;
            size_t j_best = 0;
            for (size_t j = 0; j < y_prior.points.size(); j++) {
                double score_j;
                if (config.use_feature_model) {
                    score_j = feature_model.evaluateProbabilitySingle(y_new.points[i], y_prior.points[j]);
                } else {
                    // Score it as if we have a gaussian with idenity covariance, so we get scores
                    // over the range [0, 1] with small difference -> 1, the same as if using probability
                    score_j = std::exp(-(y_new.points[i].descriptor - y_prior.points[j].descriptor).squaredNorm());
                }
                if (score_j > score_best) {
                    score_best = score_j;
                    j_best = j;
                }
            }
            if (score_best > config.correspondance_p_threshold) {
                c.push_back(Correspondance(j_best, i, score_best));
            }
        }
    }

private:
    Config config;
    const FeatureModel& feature_model;
};

#endif
