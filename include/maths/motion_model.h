#ifndef MATHS_MODELS_H
#define MATHS_MODELS_H

#include <random>

#include "maths/geometry.h"
#include "algorithm/state_estimate.h"
#include "maths/probability.h"


// ===== How the motion model works =====
// 
// Input = Twist estimate (velocity * dt) over the given interval.
// This can come from (mean velocity * dt) or (J * delta_q)
// 
// The input has no associated uncertainty. If we receive input from
// encoders, they have no uncertainty. If we receive it from a state
// estimator, the uncertainties used there define the relative confidence
// of different measurements, while the absolute uncertainty is less
// important.
//
// In both cases, it makes more sense to leave uncertainty calculation to
// the motion model. This will bound the uncertainty.
// Additionally, the variance in the twist estimate is less important than
// the variation in velocity throughout the interval. Non-uniform velocity
// gives a different relative transform.
//
// Therefore, the motion model uses the odometry model outlined in
// probabilistic robotics to model the uncertainty in relative transform,
// when this is found from a measure of twist (ie: mean velocity).
//
// This derives from the observations that:
// - If angle is fixed, linear velocity varies, then the displacement
//   only scales. Therefore linear velocity variance adds to distance.
// - If distance is fixed angular velocity varies, then this affects the
//   angle of the displacement as well as the final angle. The final value
//   of these will likely deviate from the ideal values in a perfect twist.
//   Therefore, add variance to the angle of the displacement and the
//   final angle.

class MotionModel {
public:
    struct Config {
        struct {
            double d_d;
            double d_phi1;
            double d_phi2;
            double phi1_d;
            double phi1_phi1;
            double phi2_d;
            double phi2_phi2;
        } var_weights;
        Config(): var_weights( {0, 0, 0, 0, 0, 0} ) {}
    };
    void setConfig(const Config& config) { this->config = config; }

    virtual StateEstimateGaussian getGaussian(const StateEstimateGaussian& x_prev, const Velocity& twist)const
    {
        StateEstimateGaussian dT = getTwistTransformStateEstimate(twist);

        // Linearise the state update equation about the mean x_prev and twist transform
        Eigen::Matrix3d A, B;
        A.setIdentity();
        A.block<2,1>(0,2) = x_prev.pose.rotation() * (-1)*crossProductMatrix(dT.pose.position());
        B.setIdentity();
        B.block<2,2>(0,0) = x_prev.pose.rotation();

        // Find the mean of the next state with the non-linear function,
        // and use the linearised version to get the covariance.
        StateEstimateGaussian x;
        x.pose.setFromTransform(x_prev.pose.transform() * dT.pose.transform());
        x.covariance = A*x_prev.covariance*A.transpose() + B*dT.covariance*B.transpose();

        return x;
    }

    virtual Pose sample(const Pose& x_prev, const Velocity& twist)const
    {
        Pose dT = sampleTwistTransform(twist);
        Pose x;
        x.setFromTransform(x_prev.transform() * dT.transform());
        return x;
    }

    virtual double evaluateProbability(const Pose& x, const Pose& x_prev, const Velocity& twist)
    {
        Pose dT;
        dT.setFromTransform(x_prev.transform().inverse() * x.transform());
        return evaluateTwistTransformProbability(dT, twist);
    }

private:
    Config config;

    struct Params {
        double d;
        double phi1;
        double phi2;
    };

    Params poseToParams(const Pose& dT)const
    {
        Params params;
        params.d = dT.position().norm();
        params.phi1 = std::atan2(dT.position().y(), dT.position().x());
        params.phi2 = dT.orientation() - params.phi1;
    }

    Params getParamsVariances(const Params& params)const
    {
        Params vars;
        vars.d =
            config.var_weights.d_d * params.d +
            config.var_weights.d_phi1 * params.phi1 +
            config.var_weights.d_phi2 * params.phi2;
        vars.phi1 =
            config.var_weights.phi1_d * params.d +
            config.var_weights.phi1_phi1 * params.phi1;
        vars.phi2 =
            config.var_weights.phi2_d * params.d +
            config.var_weights.phi2_phi2 * params.phi2;

        return vars;
    }

    Pose paramsToPose(const Params& params)const
    {
        Pose dT;
        dT.position() = Eigen::Rotation2Dd(params.phi1) * Eigen::Vector2d(params.d, 0);
        dT.orientation() = params.phi1 + params.phi2;
        return dT;
    }

    StateEstimateGaussian getTwistTransformStateEstimate(const Velocity& twist)const
    {
        Pose dT = twistToTransform(twist);
        Params params = poseToParams(dT);
        Params vars = getParamsVariances(params);

        StateEstimateGaussian dT_estimate;
        dT_estimate.pose = dT;

        Eigen::Matrix3d A; // delta_x = A delta_params
        A.setZero();
        A.block<2,1>(0,0) = getDirection(vars.phi1);
        A.block<2,1>(0,1) = -crossProductMatrix(dT.position());
        A(2,1) = 1;
        A(2,2) = 1;

        Eigen::Matrix3d params_cov = Eigen::Vector3d(
            vars.d, vars.phi1, vars.phi2
        ).asDiagonal();

        dT_estimate.covariance = A * params_cov * A.transpose();

        return dT_estimate;
    }

    Pose sampleTwistTransform(const Velocity& twist)const
    {
        Pose dT_mean = twistToTransform(twist);
        Params params_mean = poseToParams(dT_mean);
        Params params_vars = getParamsVariances(params_mean);

        Params params;
        params.d = sampleGaussian(params_mean.d, params_vars.d);
        params.phi1 = sampleGaussian(params_mean.phi1, params_vars.phi1);
        params.phi2 = sampleGaussian(params_mean.phi2, params_vars.phi2);

        Pose dT = paramsToPose(params);
        return dT;
    }

    double evaluateTwistTransformProbability(const Pose& dT, const Velocity& twist)
    {
        Pose dT_mean = twistToTransform(twist);
        Params params_mean = poseToParams(dT_mean);
        Params params_vars = getParamsVariances(params_mean);

        Params params = poseToParams(dT);

        double p = 1;
        p *= evaluateGaussian(params.d, params_mean.d, params_vars.d);
        p *= evaluateGaussian(params.phi1, params_mean.phi1, params_vars.phi1);
        p *= evaluateGaussian(params.phi2, params_mean.phi2, params_vars.phi2);
        return p;
    }
};


#endif
