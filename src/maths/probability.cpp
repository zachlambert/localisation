
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
    static std::default_random_engine generator;
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

double evaluateGaussian(double x, double mean, double var)
{
    return std::exp(-0.5*std::pow(x - mean, 2) / var) / std::sqrt(2*M_PI*var);
}

double sampleGaussian(double mean, double var)
{
    static std::default_random_engine generator;
    std::normal_distribution<double> gaussian(mean, std::sqrt(var));
    return gaussian(generator);
}

double sampleUniform(double a, double b)
{
    static std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(a, b);
    return distribution(generator);
}

double evaluateUniform(double a, double b)
{
    return 1 / (b - a);
}

int samplePoisson(double rate)
{
    static std::default_random_engine generator;
    std::poisson_distribution<int> distribution(rate);
    return distribution(generator);
}

double sampleExponential(double scale)
{
    static std::default_random_engine generator;
    std::exponential_distribution<double> distribution(1 / scale);
    return distribution(generator);
}

double evaluateExponential(double x, double scale)
{
    return 1 / (1 - std::exp(-x / scale));
}
