#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>


Eigen::Vector2d get_S(const Eigen::Vector2d &vector);
Eigen::Matrix2d get_S(double scalar);

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
    Eigen::Matrix2d get_R()const;
    // Homogeneous transformation
    Eigen::Isometry2d get_T()const;
    // Spatial velocity transform
    Eigen::Matrix3d get_X()const;
    void set_from_T(const Eigen::Isometry2d &T);

private:
    Eigen::Vector3d x;
};


Pose operator*(const Pose& lhs, const Pose& rhs);

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

    Velocity& operator*=(double s) {
        V *= s;
        return *this;
    }

private:
    Eigen::Vector3d V;
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

Pose twist_to_transform(const Velocity& twist);

Velocity transform_to_twist(const Pose& pose);

Eigen::Vector2d transform_point(const Pose& pose, const Eigen::Vector2d& point);

inline Eigen::Vector2d get_direction(double angle)
{
    return Eigen::Vector2d(std::cos(angle), std::sin(angle));
}

#endif
