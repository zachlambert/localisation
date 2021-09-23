
#include "maths/motion_model.h"
#include "maths/probability.h"


double MotionModel::evaluateForward(const Pose& x, const Pose &x_prev, const Velocity& u)const
{
    Pose dT;
    dT.setFromTransform(x_prev.transform().inverse() * x.transform());
    Velocity twist_mean = transformToTwist(x_prev.transform().inverse() * x.transform());
    Velocity twist = u * dt;
    return evaluateGaussian(twist.state(), twist_mean.state(), twistCovariance(twist_mean));
}

Pose MotionModel::sampleForward(const Pose& x_prev, const Velocity& u)const
{
    Velocity twist_mean = u * dt;
    Velocity twist;
    twist.state() = sampleGaussian(twist_mean.state(), twistCovariance(twist_mean));
    Pose x;
    x.setFromTransform(x_prev.transform() * twistToTransform(twist).transform());
    return x;
}


MotionModel::LinearModel MotionModel::getLinearModel(const Pose& x0_prev, const Velocity& u0)const
{
    LinearModel model;

    Velocity mean_twist = u0 * dt;
    Pose dT = twistToTransform(mean_twist);
    Pose dT_half = twistToTransform(mean_twist * 0.5);

    model.x0.setFromTransform(x0_prev.transform() * dT.transform());

    model.A = Eigen::Matrix3d::Identity();
    model.B = dT.velocityTransform() * dt;
    model.B.block<2,2>(0,0) = model.x0.rotation() * model.B.block<2,2>(0,0);

    Eigen::Matrix3d half_B = dT_half.velocityTransform() * dt;
    half_B.block<2,2>(0,0) = model.x0.rotation() * half_B.block<2,2>(0,0);
    model.Q = half_B * twistCovariance(mean_twist) * half_B.transpose();

    return model;
}


// p(u_k | x_{k-1}, x_k)
double MotionModel::evaluateInverse(const Velocity& u, const Pose& x_prev, const Pose& x)const
{
    // The same if u_k has a uniform prior
    return evaluateForward(x, x_prev, u);
}

Velocity MotionModel::sampleInverse(const Pose& x_prev, const Pose& x)const
{
    Pose dT;
    dT.setFromTransform(x_prev.transform().inverse() * x.transform());
    Velocity twist_mean = transformToTwist(x_prev.transform().inverse() * x.transform());
    Velocity twist;
    twist.state() = sampleGaussian(twist_mean.state(), twistCovariance(twist_mean));
    return twist;
}


Eigen::Matrix3d MotionModel::twistCovariance(const Velocity& mean_twist)const
{
    return Eigen::Vector3d(
        std::pow(mean_twist.linear().x(), 2) * vx,
        std::pow(mean_twist.linear().y(), 2) * vy,
        std::pow(mean_twist.angular(), 2) * vtheta
    ).asDiagonal();
}
