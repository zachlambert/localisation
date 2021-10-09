#ifndef MATHS_FEATURE_MODEL_H
#define MATHS_FEATURE_MODEL_H

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "maths/probability.h"
#include "algorithm/gaussian_methods.h"


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

    void sample(
        const PointCloud& measurements,
        const PointCloud& known_features,
        const std::vector<bool>& indicators,
        PointCloud& observed_features)const
    {
        observed_features.points.clear();

        if (known_features.points.empty()) return;
        const size_t DESCRIPTOR_SIZE = known_features.points[0].descriptor.size();

        for (size_t i = 0; i < known_features.points.size(); i++) {
            if (!indicators[i]) continue;

            // Random probability of missing a feature (false negative)
            if (sampleUniform(0, 1) < config.false_negative_p) {
                continue;
            }

            // Otherwise, add gaussian noise to corresponding known feature
            Point feature = known_features.points[i];

            feature.range += sampleGaussian(0, config.range_var);
            feature.angle += sampleGaussian(0, config.angle_var);

            for (size_t j = 0; j < feature.descriptor.size(); j++) {
                double& value = feature.descriptor(j);
                value += sampleGaussian(0, config.descriptor_var);
                if (value < 0) value = 0;
            }
            // Renormalise so max value = 1
            feature.descriptor /= feature.descriptor.sum();

            observed_features.points.push_back(feature);
        }

        // Random number of false positives, using poission distribution
        int num_false_positives = samplePoisson(config.false_positive_rate);
        for (size_t i = 0; i < num_false_positives; i++) {
            // Pick measurement randomly
            const Point& measurement = measurements.points[rand() % measurements.points.size()];
            // Random landmark descriptor
            Eigen::VectorXd descriptor = randomDescriptor(DESCRIPTOR_SIZE);
            observed_features.points.push_back(Point(measurement.range, measurement.angle, descriptor));
        }
    }

    double evaluateProbability(const Point& y_prior, const Point& y)const
    {
        // NOTE: Ignores false positives and negatives. Assumes match is correct.

        double p = 1;
        p *= evaluateGaussian(y.range, y_prior.range, config.range_var);
        double angle_dif = y.angle - y_prior.angle;
        normaliseAngle(angle_dif);
        p *= evaluateGaussian(0, angle_dif, config.angle_var);
        for (size_t j = 0; j < y_prior.descriptor.size(); j++) {
            p *= evaluateGaussian(y.descriptor(j), y_prior.descriptor(j), config.descriptor_var);
        }
        return p;
    }

    LinearModelUpdate<3, Eigen::Dynamic> linearise(const Pose& x_prior, const Point& y_prior, const Point& y)const
    {
        LinearModelUpdate<3, Eigen::Dynamic> linear_model;
        size_t Ny = y_prior.state().size();

        linear_model.C.resize(Ny, 3);
        linear_model.R.resize(Ny, Ny);
        linear_model.R.setZero();

        Eigen::VectorXd dir = Eigen::Rotation2Dd(x_prior.orientation()) * y_prior.pos();
        dir.normalize();

        linear_model.C.setZero();
        linear_model.C.block(0, 0, 1, 2) = - dir.transpose();
        linear_model.C.block(1, 0, 1, 2) = - (crossProductMatrix(1) * dir).transpose() / y_prior.range;
        linear_model.C(1, 2) = -1;

        linear_model.R(0, 0) = config.range_var;
        linear_model.R(1, 1) = config.angle_var;
        linear_model.R.block(2, 2, Ny-2, Ny-2).setIdentity();
        linear_model.R.block(2, 2, Ny-2, Ny-2) *= config.descriptor_var;

        linear_model.innovation = y.state() - y_prior.state();
        normaliseAngle(linear_model.innovation(1));

        return linear_model;
    }

private:
    Config config;
};

#endif
