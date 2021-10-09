#ifndef ALGORITHM_FEATURE_MATCHER
#define ALGORITHM_FEATURE_MATCHER

#include "maths/feature_model.h"


class FeatureMatcher {
public:
    FeatureMatcher(const FeatureModel& feature_model):
        feature_model(feature_model)
    {}

    struct Config {
        bool use_feature_model; // Otherwise just nearest neighbour search of descriptors
        double correspondance_p_threshold;
    };
    void setConfig(const Config& config) { this->config = config; }

    struct Match {
        size_t index_known;    // Known map or previous observation
        size_t index_observed; // New observation
        double score;
        Match(size_t index_known, size_t index_observed, double score):
            index_known(index_known), index_observed(index_observed), score(score)
        {}
    };

    void getMatches(
        const PointCloud& known_features,
        const PointCloud& observed_features,
        const std::vector<bool>& indicators, // Provides prior information on which saved features are feasible matches
        std::vector<Match>& matches)
    {
        matches.clear();
        for (size_t i = 0; i < observed_features.points.size(); i++) {
            double score_best = 0;
            size_t j_best = 0;

            for (size_t j = 0; j < known_features.points.size(); j++) {
                if (!indicators[j]) continue;

                double score_j;
                if (config.use_feature_model) {
                    score_j = feature_model.evaluateProbability(known_features.points[i], observed_features.points[i]);
                } else {
                    // Score it as if we have a gaussian with identity covariance
                    score_j = std::exp(-(observed_features.points[i].descriptor - known_features.points[j].descriptor).squaredNorm());
                }
                if (score_j > score_best) {
                    score_best = score_j;
                    j_best = j;
                }
            }
            if (score_best > config.correspondance_p_threshold) {
                matches.push_back(Match(j_best, i, score_best));
            }
        }
    }

private:
    Config config;
    const FeatureModel& feature_model;
};


#endif
