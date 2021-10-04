#ifndef ALGORITHM_KALMAN_FILTER_H
#define ALGORITHM_KALMAN_FILTER_H

#include <Eigen/Core>
#include <functional>
#include "maths/geometry.h"
#include "maths/point_cloud.h"


/*  Implements functions for using the extended kalman filter (and related)
 *  in the context of state estimation for mobile robots.
 *  Therefore, uses the following specific objects for x, y, y:
 *  - x = Pose
 *  - u = Velocity (twist)
 *  - y = Point cloud (features)
 */

struct GaussianStateEstimate {
    Pose pose;
    Eigen::Matrix3d covariance;
};

struct LinearisedStateEquation
{
    Pose x; // = f(x_prev, u)
    Eigen::Matrix<double, 3, 3> Sigma_u; // Variance of twist input
    Eigen::Matrix<double, 3, 3> A;
    Eigen::Matrix<double, 3, 3> B;
    Eigen::Matrix<double, 3, 3> Q;
};

struct LinearisedObservationEquation
{
    static constexpr int Nx = 3;
    static constexpr int Ny = Eigen::Dynamic;
    Point y_prior; // = g(x_prior)
    Eigen::Matrix<double, Eigen::Dynamic, 3> C;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;
};

static void ekfPredict(
    GaussianStateEstimate& estimate,
    const LinearisedStateEquation& f)
{
    estimate.pose = f.x;
    estimate.covariance =
        f.A * estimate.covariance * f.A.transpose() +
        f.B * f.Sigma_u * f.B.transpose() +
        f.Q;
}

static void ekfUpdate(
    GaussianStateEstimate& estimate,
    const Point& y,
    const LinearisedObservationEquation& g)
{
    Eigen::MatrixXd S = g.C * estimate.covariance * g.C.transpose() + g.R;
    Eigen::Matrix<double, 3, Eigen::Dynamic> K = estimate.covariance * g.C.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd innovation = y.state() - g.y_prior.state();
    normaliseAngle(innovation(1));

    estimate.pose.state() += K * innovation;
    normaliseAngle(estimate.pose.orientation());
    estimate.covariance = (Eigen::Matrix3d::Identity() - K*g.C) * estimate.covariance;
}

static void ekfUpdateMultiple(
    GaussianStateEstimate& estimate,
    const std::vector<Point>& y,
    const std::vector<LinearisedObservationEquation>& g)
{
    // If haven't moved, covariance still at zero, no need for update.
    if (estimate.covariance.determinant() == 0) return;
    // If we have no data, return;
    if (y.empty()) return;

    size_t Ny = g[0].C.rows();

    // Easiest way to do this it to use canonical gaussian representation
    Eigen::Vector3d mu;
    mu.setZero();
    Eigen::Matrix3d omega;
    omega = estimate.covariance.inverse();

    for (size_t i = 0; i < g.size(); i++) {
        Eigen::VectorXd innovation = y[i].state() - g[i].y_prior.state();
        normaliseAngle(innovation(1));
        mu += g[i].C.transpose() * g[i].R.inverse() * innovation;
        omega += g[i].C.transpose() * g[i].R.inverse() * g[i].C;
    }

    estimate.pose.state() += omega.inverse() * mu;
    estimate.covariance = omega.inverse();
}


#endif
