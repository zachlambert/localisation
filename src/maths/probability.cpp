
#include "maths/probability.h"
#include <random>
#include <Eigen/Dense>


double evaluateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{
    double det = cov.determinant();
    size_t n = cov.rows();
    auto cov_inv = cov.inverse();
    double denominator = std::sqrt(std::pow(2*M_PI, n) * det);
    double exp_value = ((x - mean).transpose() * cov_inv * (x - mean))(0,0);
    return std::exp(exp_value) / denominator;
}

Eigen::VectorXd sampleGaussian(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{
    std::default_random_engine generator;
    std::normal_distribution<double> standard_gaussian(0, 1);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::VectorXd singular_values = svd.singularValues();
    for (size_t i = 0; i < singular_values.size(); i++) {
        singular_values(i) = std::sqrt(singular_values(i));
    }

    auto C = svd.matrixU() * singular_values.asDiagonal();

    Eigen::VectorXd x;
    x.resize(mean.size());
    for (size_t i = 0; i < x.size(); i++) {
        x(i) = standard_gaussian(generator);
    }

    return mean + C*x;
}
