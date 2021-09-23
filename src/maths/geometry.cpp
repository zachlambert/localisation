
#include "maths/geometry.h"


// S is the matrix used to apply the cross product
// In 2D, we don't have a "proper" cross product, but we take the cross product
// between a 2d vector and an axis "out of the page", in the z-axis only.
// Thus, the cross product simply rotates 90 deg anticlockwise and scales.

// For vector a and scalar s, which give the cross product b, they need to
// satisfy:
// a x sk = S(a)s = b -> S(a) is a 2x1 matrix (vector)
// sk x a = S(s)a = b -> S(s) is a 2x2 matrix

Eigen::Vector2d crossProductMatrix(const Eigen::Vector2d &vector)
{
    // S(v)s = v x sk = - sk x v = rotate clockwise
    return Eigen::Vector2d(vector(1), -vector(0));
}

Eigen::Matrix2d crossProductMatrix(double scalar)
{
    // Rotate anticlockwise, and scale by scalar
    Eigen::Matrix2d S;
    S << 0, -scalar,
         scalar,  0;
    return S;
}


// Rotation matrix
Eigen::Matrix2d Pose::rotation()const
{
    Eigen::Rotation2Dd asdf;
    double theta = x[2];
    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);
    return R;
}

// Homogeneous transformation
Eigen::Isometry2d Pose::transform()const
{
    Eigen::Isometry2d T;
    T.setIdentity();
    T.translate(position());
    T.rotate(orientation());
    return T;
}

// Spatial velocity transform
Eigen::Matrix3d Pose::velocityTransform()const
{
    Eigen::Matrix3d X;
    X.topLeftCorner(2,2) = rotation().transpose();
    X.topRightCorner(2,1) = -rotation().transpose() * crossProductMatrix(position());
    X.bottomLeftCorner(1,2).setZero();
    X.bottomRightCorner(1,1).setIdentity();
    return X;
}

void Pose::setFromTransform(const Eigen::Isometry2d &T)
{
    position() = T.translation();
    Eigen::Matrix2d R = T.rotation().matrix();
    orientation() = std::atan2(R(1,0), R(0,0));
}

Pose operator*(const Pose& lhs, const Pose& rhs)
{
    Eigen::Isometry2d T = lhs.transform() * rhs.transform();
    Pose pose;
    pose.setFromTransform(T);
    return pose;
}

Pose twistToTransform(const Velocity& twist)
{
    Pose pose;
    if (twist.angular() == 0) {
        pose.position() = twist.linear();
        return pose;
    }
    double theta = twist.angular();
    Eigen::Vector2d v_hat = twist.linear() / theta;
    auto S = crossProductMatrix(1);
    pose.position() = v_hat*std::sin(theta) + S*v_hat*(1-std::cos(theta));
    pose.orientation() = twist.angular();
    return pose;
}

Velocity transformToTwist(const Pose& pose)
{
    Velocity vel;
    double theta = pose.orientation();
    if (theta == 0) {
        vel.linear() = pose.position();
        return vel;
    }

    Eigen::Matrix2d M = (std::sin(theta)/(1-std::cos(theta))) * Eigen::Matrix2d::Identity();
    auto S = crossProductMatrix(1);
    vel.linear() = 0.5*(M - S)*pose.position();
    vel.angular() = theta;
    return vel;
}

Eigen::Vector2d transformPoint(const Pose& pose, const Eigen::Vector2d& point)
{
    Eigen::Vector3d result = pose.transform() * Eigen::Vector3d(point.x(), point.y(), 1);
    return Eigen::Vector2d(result.x(), result.y());
}
