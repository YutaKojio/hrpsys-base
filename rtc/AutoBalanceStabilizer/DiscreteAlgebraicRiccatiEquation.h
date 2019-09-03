// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  DiscreteAlgebraicRiccatiEquation.h
 * @brief
 *
 * Compute the solution of the discrete-time algebraic Riccati equation
 * by Arimoto-Potter's method
 *
 * Riccati equation:
 * X = A'XA - (A'XB)(R + B'XB)^{-1}(B'XA) + Q
 *
 * @date  $Date$
 */

#ifndef DISCRETE_ALGEBRAIC_RICCATI_EQUATION_H
#define DISCRETE_ALGEBRAIC_RICCATI_EQUATION_H

#include <Eigen/Eigenvalues>

namespace hrp
{

template<size_t dim_state, size_t dim_control>
Eigen::Matrix<double, dim_state, dim_state> solveDiscreteAlgebraicRiccati(
    const Eigen::Ref<const Eigen::Matrix<double, dim_state, dim_state>>& A,
    const Eigen::Ref<const Eigen::Matrix<double, dim_state, dim_control>>& B,
    const Eigen::Ref<const Eigen::Matrix<double, dim_state, dim_state>>& Q,
    const Eigen::Ref<const Eigen::Matrix<double, dim_control, dim_control>>& R)
{
    Eigen::Matrix<double, 2 * dim_state, 2 * dim_state> hamilton;
    hamilton <<
        A + B * R.inverse() * B.transpose() * (A.inverse()).transpose() * Q,
        -B * R.inverse() * B.transpose() * (A.inverse()).transpose(),
        -(A.inverse()).transpose() * Q,
        (A.inverse()).transpose();

    const Eigen::EigenSolver<decltype(hamilton)> ham_eigenmat(hamilton);

    Eigen::Matrix<std::complex<double>, 2 * dim_state, dim_state> eigenvecs_inside;
    const auto eigenvals = ham_eigenmat.eigenvalues();
    const auto eigenvecs = ham_eigenmat.eigenvectors();

    // Choose eigenvectors whose eigenvalues are inside the unit circle
    for (size_t i = 0, col = 0; i < 2 * dim_state; ++i) {
        if (abs(eigenvals[i]) < 1) eigenvecs_inside.col(col++) = eigenvecs.col(i);
    }

    const auto U1 = eigenvecs_inside.topRows(dim_state);
    const auto U2 = eigenvecs_inside.bottomRows(dim_state);

    return (U2 * U1.inverse()).real();
}

}

#endif // DISCRETE_ALGEBRAIC_RICCATI_EQUATION_H
