#ifndef MATHS_MODELS_H
#define MATHS_MODELS_H

#include <random>

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"


struct StateSpace
{
    Eigen::MatrixXd A, B, C, D, Q, R;
};

double evaluateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{
    double det = cov.determinant();
    size_t n = cov.rows();
    auto cov_inv = cov.inverse();
    double denominator = std::sqrt(std::pow(2*M_PI, n) * det);
    double exp_value = ((x - mean).transpose() * cov_inv * (x - mean))(0,0);
    return std::exp(exp_value) / denominator;
}

Eigen::VectorXd sampleGaussian(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{
    std::default_random_engine generator;
    std::normal_distribution<double> standard_gaussian(0, 1);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(cov);

    Eigen::VectorXd singular_values = svd.singularValues();
    for (size_t i = 0; i < singular_values.size(); i++) {
        singular_values(i) = std::sqrt(singular_values(i));
    }

    auto C = svd.matrixU() * singular_values.asDiagonal();

    Eigen::VectorXd x;
    x.resize(mean.size());
    for (size_t i = 0; i < x.size(); i++) {
        x(i) = standard_gaussian(generator);
    }

    return mean + C*x;
}


class MotionModel
{
public:
    void setTimeStep(double dt) { this->dt = dt; }
    void setVarianceScaling(double vx, double vy, double vtheta) {
        this->vx = vx;
        this->vy = vy;
        this->vtheta = vtheta;
    }

    // p(x_k | x_{k-1}, u_k)
    double evaluateForward(const Velocity& u, const Pose& x_prev, const Pose& x)const
    {
        Pose dT = Pose(x.transform().inverse() * x_prev.transform());
        Velocity twist = transformToTwist(dT);
        Velocity mean_twist = u * dt;

        Velocity error;
        error.state() = twist.state() - mean_twist.state();
        
        Eigen::Matrix3d cov = Eigen::VectorXd(
            mean_twist.linear().x() * vx,
            mean_twist.linear().y() * vy,
            mean_twist.angular() * vtheta
        ).asDiagonal();

        return evaluateGaussian(error.state(), Eigen::Vector3d::Zero(), cov);
    }

    Pose sampleForward(const Velocity& u, const Pose& x_prev)const
    {
        Velocity mean_twist = u * dt;
        Eigen::Matrix3d cov = Eigen::VectorXd(
            mean_twist.linear().x() * vx,
            mean_twist.linear().y() * vy,
            mean_twist.angular() * vtheta
        ).asDiagonal();

        Velocity sampled_twist;
        sampled_twist.state() = mean_twist.state() + sampleGaussian(Eigen::Vector3d::Zero(), cov);

        return Pose(x_prev.transform() * twistToTransform(sampled_twist));
    }

    StateSpace getLinearModel(const Pose& x0)const
    {
        StateSpace ss;
        // TODO
        return ss;
    }

    // p(u_k | x_{k-1}, x_k)
    double evaluateInverse(const Velocity& u, const Pose& x_prev, const Pose& x)
    {
        // TODO
    }

    Velocity sampleInverse(const Velocity& u, const Pose& x_prev, const Pose& x)
    {
        // TODO
    }

private:
    double dt;
    double vx, vy, vtheta;
};

#endif
