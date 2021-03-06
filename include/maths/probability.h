#ifndef MATHS_PROBABILITY_H
#define MATHS_PROBABILITY_H

#include <Eigen/Core>

double evaluateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);
Eigen::VectorXd sampleGaussian(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

double evaluateGaussian(double x, double mean, double var);
double sampleGaussian(double mean, double cov);

double sampleUniform(double a, double b);
double evaluateUniform(double a, double b);

int samplePoisson(double rate);

double sampleExponential(double scale);
double evaluateExponential(double x, double scale);

#endif
