#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <vector>
#include "maths/geometry.h"
#include "maths/probability.h"
#include "maths/state_estimate.h"


class Odometry {
public:
    virtual StateEstimateGaussian getGaussian()const = 0; 
    virtual Pose sample()const = 0;;
};

class OdometryGeneral: public Odometry {
public:
    void update(const Pose& pose)
    {
        this->pose = pose;
        variables.phi1 = std::atan2(pose.position().y(), pose.position().x());
        variables.dist = pose.position().norm();
        variables.phi2 = pose.orientation() - variables.phi1;

        variables.variances = Eigen::Vector3d(
            // phi1 variance
            params[0]*std::pow(variables.phi1, 2) + params[1]*std::pow(variables.dist, 2),
            // dist variance
            params[2]*std::pow(variables.dist, 2) + params[3]*(std::pow(variables.phi1, 2) + std::pow(variables.phi2, 2)),
            // phi2 variance
            params[0]*std::pow(variables.phi2, 2) + params[1]*std::pow(variables.dist, 2)
        );
    }

    void setVarianceParameters(double params[4])
    {
        for (size_t i = 0; i < 4; i++) {
            this->params[i] = params[i];
        }
    }

    virtual StateEstimateGaussian getGaussian()const
    {
        StateEstimateGaussian gaussian;
        gaussian.pose = pose;

        // Linearised function from variables -> delta x
        Eigen::Matrix3d A;
        A.setIdentity();
        A.block<2,1>(0,0) = Eigen::Rotation2Dd(variables.phi2) * Eigen::Vector2d(0, variables.dist);
        A.block<2,1>(0,1) = Eigen::Rotation2Dd(variables.phi2) * Eigen::Vector2d(1, 0);
        A(2,2) = 1;

        Eigen::Matrix3d variables_cov = variables.variances.asDiagonal();
        gaussian.covariance = A*variables_cov*A.transpose();
        return gaussian;
    }

    virtual Pose sample()const
    {
        double phi1 = sampleGaussian(variables.phi1, variables.variances(0));
        double dist = sampleGaussian(variables.dist, variables.variances(1));
        double phi2 = sampleGaussian(variables.phi2, variables.variances(2));
        Pose pose;
        pose.position() = Eigen::Rotation2Dd(phi1) * Eigen::Vector2d(dist, 0);
        pose.orientation() = phi1 + phi2;
        return pose;
    }

private:
    double params[4]; // Determines variance of variables

    // Alternative parameterisation of pose
    struct Variables {
        double phi1, dist, phi2;
        Eigen::Vector3d variances;
    };

    Pose pose;
    Variables variables;
    Eigen::Matrix3d covariance;
};

#endif
