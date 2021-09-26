#ifndef MATHS_STATE_ESTIMATE_H
#define MATHS_STATE_ESTIMATE_H

#include <vector>
#include "maths/geometry.h"


struct StateEstimateGaussian {
    Pose pose;
    Eigen::Matrix3d covariance;
};

struct StateEstimateParticles {
    struct Particle {
        Pose pose;
        double weight;
    };
    std::vector<Particle> particles;
};

struct StateEstimateGaussianMixture {
    struct Component {
        StateEstimateGaussian gaussian;
        double weight;
    };
    std::vector<Component> components;
};

#endif
