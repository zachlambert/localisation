#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "maths/probability.h"
#include "state/terrain.h"
#include "algorithm/gaussian_methods.h"
#include <iostream>


class MeasurementModel {
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


#endif
