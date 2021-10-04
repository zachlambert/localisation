#ifndef MATHS_GEOMETRY_H
#define MATHS_GEOMETRY_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>


Eigen::Vector2d crossProductMatrix(const Eigen::Vector2d &vector);
Eigen::Matrix2d crossProductMatrix(double scalar);

class Pose {
public:
    Pose() {
        x.setZero();
    }
    Pose(const Eigen::Isometry2d& transform)
    {
        setFromTransform(transform);
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
    Eigen::Matrix2d rotation()const;
    // Homogeneous transformation
    Eigen::Isometry2d transform()const;
    // Spatial velocity transform
    Eigen::Matrix3d velocityTransform()const;
    void setFromTransform(const Eigen::Isometry2d &T);

private:
    Eigen::Vector3d x;
};

class Velocity: public Eigen::Vector3d {
public:
    Velocity()
    {
        setZero();
    }

    // Access position
    const Eigen::VectorBlock<const Eigen::Vector3d, 2> linear()const {
        return head<2>();
    }
    Eigen::VectorBlock<Eigen::Vector3d, 2> linear() {
        return head<2>();
    }

    // Access orientation
    const double angular()const {
        return (*this)(2);
    }
    double &angular() {
        return (*this)(2);
    }

    Pose getTwistTransform()const;
    void setFromTwistTransform(const Pose& pose);

    // Required when inheriting
    template<typename OtherDerived>
    Velocity(const Eigen::MatrixBase<OtherDerived>& other): Eigen::Vector3d(other) {}
    template<typename OtherDerived>
    Velocity& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector3d::operator=(other);
        return *this;
    }
};

static Velocity operator*(const Velocity& lhs, double rhs)
{
    Velocity result(lhs);
    result *= rhs;
    return result;
}
static Velocity operator*(double lhs, const Velocity& rhs)
{
    Velocity result(rhs);
    result *= lhs;
    return result;
}

Eigen::Vector2d transformPoint(const Pose& pose, const Eigen::Vector2d& point);

inline Eigen::Vector2d getDirection(double angle)
{
    return Eigen::Vector2d(std::cos(angle), std::sin(angle));
}

inline void normaliseAngle(double& angle)
{
    while (angle < -M_PI) angle += 2*M_PI;
    while (angle > M_PI) angle -= 2*M_PI;
}

#endif
