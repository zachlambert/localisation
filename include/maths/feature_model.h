#ifndef MATHS_FEATURE_MODEL_H
#define MATHS_FEATURE_MODEL_H

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "maths/probability.h"
#include "state/terrain.h"
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

    double evaluateProbability(const Point& y_prior, const Point& y)const
    {
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

#if 0 // Don't think I need this
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
#endif

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
