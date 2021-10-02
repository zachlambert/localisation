#ifndef MATHS_ODOMETRY_H
#define MATHS_ODOMETRY_H

#include "maths/geometry.h"
#include "algorithm/state_estimate.h"
#include "maths/probability.h"


// I am using the term "Odometer" to refer to any object that provides
// odometry. Odometry is an estimate of the relative transform over a
// given interval, used in the prediction step of a state estimator.
// 
// It needs to provide a distribution of the relative transform, done
// by providing functions for evaluating the probability or sampling
// from the distribution.
// It also needs to provide a gaussian approximation, available for
// gaussian state estimators (ie: kalman filters).
//
// This interface allows for various methods of estimating odometry:
// - Kinematic odometry: Use joint measurements and a kinematic model
//   (eg: encoder feedback for wheeled robot or hexapod kinematic model)
// - Inertial navigation system: Uses a state estimator to track the
//   pose of the robot. When odometry is requested, the latest state estimate
//   provides the distribution of the relative transform. This may be
//   through a kalman filter, particle filter or other.
//   There are two options here:
//   - INS operates at faster rate: Maintains its own pose estimate,
//     which resets on sending to the lower rate state estimator.
//   - INS operates at same rate: Maintain a velocity estimate, return
//     the pose estimate using this. eg: 0.5*(v_prev + v).
// - Visual odometry: General category of methods that use visual
//   information (lidar, radar, sonar, cameras) to estimate relative
//   transforms. In the case of 2D scans (eg: lidar), this is done
//   by aligning point clouds (features or raw data) and it can always
//   determine the relative transform.
//   For monocular cameras, you cannot determine the absolute scale of
//   the displacement. This ambiguity is resolve if you know the depth
//   of objects (eg: can identify the ground plane and know the camera height),
//   or you know the distance between frames.
//   This is fixed in stereo systems, where two frames are provided at
//   each timestep from two cameras. The absolute displacement behind these
//   cameras is known, which: allows depth of features to be calculated,
//   allowing the absolute displacement between frames be calculated.
// - Some fusion of the above. eg: Fuse IMU measurements with visual
//   odometry. (see Intel realsense d435i depth camera)
//
// What am I doing in this project?
// - In this project, since I'm simulating everything, implementing actual
//   odometers is pointless, since the difficulty is getting this to work
//   with real sensors.
// - Therefore, I will use a fake odometer, which takes the true pose as
//   an input, then

class Odometer {
public:
    virtual StateEstimateGaussian getGaussian()const = 0;
    virtual Pose sample()const = 0;
};

class Odometry: public Odometer {
public:
    void update(const Pose& u)
    {
        this->u = u;

        state.phi1 = std::atan2(u.position().y(), u.position().x());
        state.dist = u.position().norm();
        state.phi2 = u.orientation() - state.phi1;

        double phi1_var =
            variance_gains[0] * std::pow(state.phi1, 2) +
            variance_gains[1] * std::pow(state.dist, 2);

        double dist_var =
            variance_gains[2] * std::pow(state.dist, 2) +
            variance_gains[3] * std::pow(state.phi1, 2) +
            variance_gains[3] * std::pow(state.phi2, 2);

        double phi2_var =
            variance_gains[0] * std::pow(state.phi2, 2) +
            variance_gains[1] * std::pow(state.dist, 2);

        state.variances(0) = phi1_var;
        state.variances(1) = dist_var;
        state.variances(2) = phi2_var;
    }

    void setVarianceGains(double variance_gains[4])
    {
        for (size_t i = 0; i < 4; i++) {
            this->variance_gains[i] = variance_gains[i];
        }
    }

    virtual StateEstimateGaussian getGaussian()const
    {
        StateEstimateGaussian gaussian;
        gaussian.pose = u;

        // Linearised function from state -> u
        Eigen::Matrix3d A;
        A.setIdentity();
        A.block<2,1>(0,0) = Eigen::Rotation2Dd(state.phi2) * Eigen::Vector2d(0, state.dist);
        A.block<2,1>(0,1) = Eigen::Rotation2Dd(state.phi2) * Eigen::Vector2d(1, 0);
        A(2,2) = 1;

        Eigen::Matrix3d state_cov = state.variances.asDiagonal();
        gaussian.covariance = A*state_cov*A.transpose();
        return gaussian;
    }

    virtual Pose sample()const
    {
        double phi1 = sampleGaussian(state.phi1, state.variances(0));
        double dist = sampleGaussian(state.dist, state.variances(1));
        double phi2 = sampleGaussian(state.phi2, state.variances(2));
        Pose pose;
        pose.position() = Eigen::Rotation2Dd(phi1) * Eigen::Vector2d(dist, 0);
        pose.orientation() = phi1 + phi2;
        return pose;
    }

private:
    double variance_gains[4]; // Determines variance of state

    // Alternative parameterisation of pose
    struct State {
        double phi1, dist, phi2;
        Eigen::Vector3d variances;
    };

    Pose u;
    State state; // Parametrisation of u
};

#endif
