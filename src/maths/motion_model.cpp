
#include "maths/motion_model.h"
#include "maths/probability.h"


double MotionModel::evaluateForward(const Pose& x, const Pose &x_prev, const Odometry& u)const
{
    Pose dT;
    dT.setFromTransform(x_prev.transform().inverse() * x.transform());
    return evaluateGaussian(dT.state(), u.pose.state(), u.covariance);
}

Pose MotionModel::sampleForward(const Pose& x_prev, const Odometry& u)const
{
    Pose dT;
    dT.state() = sampleGaussian(u.pose.state(), u.covariance);
    Pose x;
    x.setFromTransform(x_prev.transform() * dT.transform());
    return x;
}


MotionModel::LinearModel MotionModel::getLinearModel(const Pose& x0_prev, const Odometry& u0)const
{
    LinearModel model;
    model.x0.setFromTransform(x0_prev.transform() * u0.pose.transform());

    model.A = Eigen::Matrix3d::Identity();
    model.B.block<2,2>(0,0) = model.x0.rotation();
    // model.Q Left at zero

    return model;
}
