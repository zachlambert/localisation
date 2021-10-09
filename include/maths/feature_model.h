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
    };

    void setConfig(const Config& config) { this->config = config; }

    Point sample(const Pose& pose, const Point& known_feature)const
    {
        double range = known_feature.range(pose) + sampleGaussian(0, config.range_var);
        double angle = known_feature.angle(pose) + sampleGaussian(0, config.angle_var);

        Point feature;
        feature.setPolar(range, angle);

        for (size_t j = 0; j < feature.descriptor.size(); j++) {
            double& value = feature.descriptor(j);
            value += sampleGaussian(0, config.descriptor_var);
            if (value < 0) value = 0;
        }
        // Renormalise so max value = 1
        feature.descriptor /= feature.descriptor.sum();

        return feature;
    }

    double evaluateProbability(const Pose& x, const Point& y_prior, const Point& y)const
    {
        // y_prior given in global frame. Need to pass pose estimate x to move to this frame.
        // y given in measurement frame, aligned with pose estimate.

        double p = 1;
        p *= evaluateGaussian(y.range(), y_prior.range(x), config.range_var);
        double angle_dif = y.angle() - y_prior.angle(x);
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

        Eigen::VectorXd dir = Eigen::Rotation2Dd(x_prior.orientation()) * (y_prior.pos - x_prior.position());
        dir.normalize();

        linear_model.C.setZero();
        linear_model.C.block(0, 0, 1, 2) = - dir.transpose();
        linear_model.C.block(1, 0, 1, 2) = - (crossProductMatrix(1) * dir).transpose() / y_prior.range(x_prior);
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
