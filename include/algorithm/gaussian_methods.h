#ifndef ALGORITHM_KALMAN_FILTER_H
#define ALGORITHM_KALMAN_FILTER_H

#include <vector>
#include <Eigen/Core>


template <int Nx, int Nu>
struct LinearModelPredict {
    Eigen::Matrix<double, Nx, 1> x_next;
    Eigen::Matrix<double, Nx, Nx> A;
    Eigen::Matrix<double, Nx, Nu> B;
    Eigen::Matrix<double, Nx, Nx> Q;
    Eigen::Matrix<double, Nu, Nu> Sigma_u;
};

template <int Nx, int Nu>
static void ekfPredict(
    Eigen::Matrix<double, Nx, 1>& x,
    Eigen::Matrix<double, Nx, Nx>& cov,
    const LinearModelPredict<Nx, Nu>& lm)
{
    x = lm.x_next;
    cov = lm.A*cov*lm.A.transpose() + lm.B*lm.Sigma_u*lm.B.transpose() + lm.Q;
}


template <int Nx, int Ny>
struct LinearModelUpdate {
    Eigen::Matrix<double, Ny, 1> innovation;
    Eigen::Matrix<double, Ny, Nx> C;
    Eigen::Matrix<double, Ny, Ny> R;
};

template <int Nx, int Ny>
static void ekfUpdate(
    Eigen::Matrix<double, Nx, 1>& x,
    Eigen::Matrix<double, Nx, Nx>& cov,
    const LinearModelUpdate<Nx, Ny>& lm)
{
    Eigen::Matrix<double, Ny, Ny> S = lm.C*cov*lm.C.transpose() + lm.R;
    Eigen::Matrix<double, Nx, Ny> K = cov * lm.C.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();

    x += K * lm.innovation;
    cov = (Eigen::Matrix3d::Identity() - K*lm.C) * cov;
}


template <int Nx, int Ny>
static void ekfUpdateMultiple(
    Eigen::Matrix<double, Nx, 1>& x,
    Eigen::Matrix<double, Nx, Nx>& cov,
    const std::vector<LinearModelUpdate<Nx, Ny>>& lms)
{
    // If the current estimate has zero covariance, the result will be unchanged
    // Also: the later code can't handle this case, since it needs to invert the covariance.
    if (cov.determinant() == 0) return;

    // If we have no data, return;
    if (lms.empty()) return;

    // To perform an update step with multiple observations, best to use the
    // canonical gaussian representation. ie: Fuse the gaussians. Equivalent to information filter update step.

    // Since everything is linearised, we work with deviations from the prior state
    // Therefore, the prior estimate = 0, so initial mu = 0.
    // However, we set the initial information matrix from the prior covariance.
    Eigen::Matrix<double, Nx, 1> eta;
    eta.setZero();
    Eigen::Matrix<double, Nx, Nx> Omega;
    Omega = cov.inverse();

    for (size_t i = 0; i < lms.size(); i++) {
        const LinearModelUpdate<Nx, Ny>& lm = lms[i];
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Rinv = lm.R.inverse();
        eta += lm.C.transpose() * Rinv * lm.innovation;
        Omega += lm.C.transpose() * Rinv * lm.C;
    }

    // The final state is the optimal value of (x - x_prior) so add this to prior state
    cov = Omega.inverse();
    x += cov * eta;
}


template <int Nx, int Ny>
void ekfUpdateMultipleWithLikelihood(
    Eigen::Matrix<double, Nx, 1>& x,
    Eigen::Matrix<double, Nx, Nx>& cov,
    double& log_likelihood,
    const std::vector<LinearModelUpdate<Nx, Ny>>& lms)
{
    typedef Eigen::Matrix<double, Nx, 1> x_t;
    typedef Eigen::Matrix<double, Nx, Nx> cov_t;

    if (cov.determinant() == 0) return;
    if (lms.empty()) return;

    x_t eta;
    eta.setZero();
    cov_t Omega;
    Omega = cov.inverse();

    double k = 0;

    for (size_t i = 0; i < lms.size(); i++) {
        const LinearModelUpdate<Nx, Ny>& lm = lms[i];

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Rinv = lm.R.inverse();

        x_t eta_i = lm.C.transpose() * Rinv * lm.innovation;
        eta += eta_i;
        cov_t Omega_i = lm.C.transpose() * Rinv * lm.C;
        Omega += Omega_i;

        // Omega_i may be singular (if rank(C) < state dimension), so need to be careful with the below part.
        // 1. Use the pseudoinverse of Omega to get Sigma.
        // 2. Use the product of non-zero eigenvectors instead of determinant.
        // Need to look at maths in more detail for handling this degenerate case, but my hand-wavy reasoning
        // is that the unobservable state component gets infinite variance. This has no effect on the probability.
        // The effect on k would be (+infinity -infinity = 0), but we can't represent this with floats.
        // Threfore, ignore the unobservable state completely by using the pseudoinverse and ignoring factor of 0 in the determinant.


        Eigen::CompleteOrthogonalDecomposition<cov_t> decomposition = Omega_i.completeOrthogonalDecomposition();

        cov_t Sigma_i = decomposition.pseudoInverse();
        k += eta_i.transpose() * Sigma_i * eta_i;

        double nonSingularDeterminant = std::abs(decomposition.matrixT().determinant());
        k += -std::log(nonSingularDeterminant) + Ny*std::log(2*M_PI);
        // Equivalent to: denominator *= std::sqrt(Sigma_i.determinant() * two_pi_pow_Ny);
    }

    cov = Omega.inverse();
    x += cov * eta;

    k -= eta.transpose() * cov * eta;
    k -= std::log(cov.determinant()) + Ny*std::log(2*M_PI);

    log_likelihood -= 0.5*k;
}

#endif
