#ifndef MATHS_MODELS_H
#define MATHS_MODELS_H

#include <random>

#include "maths/odometry.h"


class MotionModel
{
public:
    void setKidnapProbability(double epsilon) { this->epsilon = epsilon; }

    void predictGaussian(StateEstimateGaussian& x, const StateEstimateGaussian& x_prev, const Odometry& u)const
    {
        // Note: Allow the same x to be provided as x and x_prev.

        StateEstimateGaussian delta_x = u.getGaussian();

        Eigen::Matrix3d A, B;
        A.setIdentity();
        A.block<2,1>(0,2) = x_prev.pose.rotation() * (-1)*crossProductMatrix(delta_x.pose.position());
        B.setIdentity();
        B.block<2,2>(0,0) = x_prev.pose.rotation();

        x.pose.setFromTransform(x_prev.pose.transform() * delta_x.pose.transform());
        x.covariance = A*x_prev.covariance*A.transpose() + B*delta_x.covariance*B.transpose();
    }

    void predictParticleFilter(StateEstimateParticles& x, const StateEstimateParticles& x_prev, const Odometry& u)const
    {
        // Note: Allow the same x to be provided as x and x_prev.

        x.particles.resize(x_prev.particles.size());
        for (size_t i = 0; i < x_prev.particles.size(); i++) {
            if (sampleKidnap()) {
                x.particles[i].pose = sampleKidnappedGaussian(x_prev.particles[i].pose);
            } else {
                // TODO: Since we are sampling from the true p(delta_x) the
                // incremental weight is 1. ie: Don't need to change weight.
                // May want to allow sampling from a general q(delta_x), for which we need
                // to be able to evaluate p(delta_x) from odometry.
                Pose delta_x = u.sample();
                x.particles[i].pose.setFromTransform(x_prev.particles[i].pose.transform() * delta_x.transform());
                x.particles[i].weight = x_prev.particles[i].weight;
            }
        }
    }

    void predictGaussianMixture(StateEstimateGaussianMixture& x, const StateEstimateGaussianMixture& x_prev, const Odometry& u)const
    {
        // TODO: Check this is correct
        x.components.resize(x_prev.components.size());
        for (std::size_t i = 0; i < x_prev.components.size(); i++) {
            x.components[i].weight = x_prev.components[i].weight;
            if (sampleKidnap()) {
                x.components[i].gaussian = sampleKidnappedGaussian(x_prev.components[i].gaussian.pose);
            } else {

            }
        }
    }

private:
    bool sampleKidnap()const {
        return sampleUniform(0, 1) > epsilon ? false : true;
    }
    Pose sampleKidnappedPose(const Pose& initial)const
    {
        Pose pose;
        // TODO: Do something here. Either sample somewhere near the initial position
        // or take the map as an extra input and use this somehow.
        return pose;
    }
    Eigen::Matrix3d getKidnappedCovariance()const
    {
        Eigen::Matrix3d covariance;
        return covariance;
    }

    double epsilon;
};

#endif
