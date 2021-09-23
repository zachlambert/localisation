#ifndef MATHS_PROBABILITY_H
#define MATHS_PROBABILITY_H

#include <Eigen/Core>


double evaluateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);
Eigen::VectorXd sampleGaussian(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

#endif
