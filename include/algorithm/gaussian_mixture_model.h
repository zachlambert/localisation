#ifndef ALGORITHM_GAUSSIAN_MIXTURE_MODEL_H
#define ALGORITHM_GAUSSIAN_MIXTURE_MODEL_H

#include "kalman_filter.h"

struct GaussianMixtureStateEstimate {
    struct Component {
        GaussianStateEstimate estimate;
        double probability;
    };
    std::vector<Component> components;
};



#endif
