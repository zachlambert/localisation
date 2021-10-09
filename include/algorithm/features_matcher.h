#ifndef ALGORITHM_FEATURE_MATCHER
#define ALGORITHM_FEATURE_MATCHER



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

    void getCorrespondances(std::vector<Correspondance>& c, const PointCloud& y_new, const PointCloud& y_prior)const
    {
        c.clear();
        for (size_t i = 0; i < y_new.points.size(); i++) {
            double score_best = 0;
            size_t j_best = 0;
            for (size_t j = 0; j < y_prior.points.size(); j++) {
                double score_j;
                if (config.use_feature_model) {
                    score_j = feature_model.evaluateProbabilitySingle(y_new.points[i], y_prior.points[j]);
                } else {
                    // Score it as if we have a gaussian with idenity covariance
                    score_j = std::exp(-(y_new.points[i].descriptor - y_prior.points[j].descriptor).squaredNorm());
                }
                if (score_j > score_best) {
                    score_best = score_j;
                    j_best = j;
                }
            }
            if (score_best > config.correspondance_p_threshold) {
                c.push_back(Correspondance(j_best, i, score_best));
            }
        }
    }

private:
    Config config;
    const FeatureModel& feature_model;
};


#endif
