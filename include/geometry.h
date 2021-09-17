#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

// S is the matrix used to apply the cross product
// In 2D, we don't have a "proper" cross product, but we take the cross product
// between a 2d vector and an axis "out of the page", in the z-axis only.
// Thus, the cross product simply rotates 90 deg anticlockwise and scales.

// For vector a and scalar s, which give the cross product b, they need to
// satisfy:
// a x sk = S(a)s = b -> S(a) is a 2x1 matrix (vector)
// sk x a = S(s)a = b -> S(s) is a 2x2 matrix

Eigen::Vector2d get_S(const Eigen::Vector2d &vector)
{
    // S(v)s = v x sk = - sk x v = rotate clockwise
    return Eigen::Vector2d(vector(1), -vector(0));
}

Eigen::Matrix2d get_S(double scalar)
{
    // Rotate anticlockwise, and scale by scalar
    Eigen::Matrix2d S;
    S << 0, -scalar,
         scalar,  0;
    return S;
}

class Pose {
public:
    Pose() {
        x.setZero();
    }

    // Access state vector
    const Eigen::Vector3d &state()const { return x; }
    Eigen::Vector3d &state() { return x; }

    // Access position
    const Eigen::VectorBlock<const Eigen::Vector3d, 2> position()const {
        return x.head<2>();
    }
    Eigen::VectorBlock<Eigen::Vector3d, 2> position() {
        return x.head<2>();
    }

    // Access orientation
    const double orientation()const {
        return x(2);
    }
    double &orientation() {
        return x(2);
    }

    // Rotation matrix
    Eigen::Matrix2d get_R()const
    {
        Eigen::Rotation2Dd asdf;
        double theta = x[2];
        Eigen::Matrix2d R;
        R << std::cos(theta), -std::sin(theta),
             std::sin(theta),  std::cos(theta);
        return R;
    }

    // Homogeneous transformation
    Eigen::Isometry2d get_T()const
    {
        Eigen::Isometry2d T;
        T.rotate(orientation());
        T.translate(position());
        return T;
    }

    // Spatial velocity transform
    Eigen::Matrix3d get_X()const
    {
        Eigen::Matrix3d X;
        X.topLeftCorner(2,2) = get_R().transpose();
        X.topRightCorner(2,1) = -get_R().transpose() * get_S(position());
        X.bottomLeftCorner(1,2).setZero();
        X.bottomRightCorner(1,1).setIdentity();
        return X;
    }

    void set_from_T(const Eigen::Isometry2d &T)
    {
        position() = T.translation();
        Eigen::Matrix2d R = T.rotation();
        orientation() = std::atan2(R(1,0), R(0,0));
    }

private:
    Eigen::Vector3d x;
};


Pose operator*(const Pose& lhs, const Pose& rhs)
{
    Eigen::Isometry2d T = lhs.get_T() * rhs.get_T();
    Pose pose;
    pose.set_from_T(T);
    return pose;
}

class Velocity {
public:
    Velocity()
    {
        V.setIdentity();
    }

    // Access state vector
    const Eigen::Vector3d &state()const { return V; }
    Eigen::Vector3d &state() { return V; }

    // Access position
    const Eigen::VectorBlock<const Eigen::Vector3d, 2> linear()const {
        return V.head<2>();
    }
    Eigen::VectorBlock<Eigen::Vector3d, 2> linear() {
        return V.head<2>();
    }

    // Access orientation
    const double angular()const {
        return V(2);
    }
    double &angular() {
        return V(2);
    }

private:
    Eigen::Vector3d V;
};


Pose twist_to_transform(const Velocity& twist)
{
    Pose pose;
    if (twist.angular() == 0) {
        pose.position() = twist.linear();
        return pose;
    }
    double theta = twist.angular();
    Eigen::Vector2d v_hat = twist.linear() / theta;
    pose.position() = v_hat*std::sin(theta) + get_S(1)*v_hat*(1-std::cos(theta));
    pose.orientation() = twist.angular();
    return pose;
}

Velocity transform_to_twist(const Pose& pose)
{
    Velocity vel;
    double theta = pose.orientation();
    if (theta == 0) {
        vel.linear() = pose.position();
        return vel;
    }

    Eigen::Matrix2d M = (std::sin(theta)/(1-std::cos(theta))) * Eigen::Matrix2d::Identity();
    vel.linear() = 0.5*(M - get_S(1))*pose.position();
    vel.angular() = theta;
    return vel;
}

#endif
