#ifndef MATHS_MODELS_H
#define MATHS_MODELS_H

#include <random>

#include "maths/geometry.h"
#include "maths/point_cloud.h"
#include "state/terrain.h"


class MotionModel
{
public:
    void setTimeStep(double dt) { this->dt = dt; }
    void setTwistVarianceScaling(double vx, double vy, double vtheta) {
        this->vx = vx;
        this->vy = vy;
        this->vtheta = vtheta;
    }

    // p(x_k | x_{k-1}, u_k)
    double evaluateForward(const Pose& x, const Pose &x_prev, const Velocity& u)const;
    Pose sampleForward(const Pose& x_prev, const Velocity& u)const;

    // p(u_k | x_{k-1}, x_k)
    double evaluateInverse(const Velocity& u, const Pose& x_prev, const Pose& x)const;
    Velocity sampleInverse(const Pose& x_prev, const Pose& x)const;

    // dx = Adx_prev + Bdu + w
    struct LinearModel
    {
        Pose x0;
        Eigen::Matrix3d A;
        Eigen::Matrix3d B;
        Eigen::Matrix3d Q;
        LinearModel(){ A.setZero(); B.setZero(); Q.setZero(); }
    };
    LinearModel getLinearModel(const Pose& x0_prev, const Velocity& u0)const;

private:
    Eigen::Matrix3d twistCovariance(const Velocity& mean_twist)const;

    double dt;
    double vx, vy, vtheta;
};

#endif
