#ifndef FAST_LIO_DCREG_HPP
#define FAST_LIO_DCREG_HPP

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_lio {
namespace dcreg {

template <typename Scalar>
struct Config {
    bool enable = false;
    bool log_enable = true;
    int log_every_n_frames = 30;
    Scalar eigenvalue_threshold = Scalar(120);
    Scalar condition_threshold = Scalar(10);
    Scalar kappa_target = Scalar(10);
    Scalar regularization_alpha = Scalar(1);
    Scalar inverse_relative_threshold = Scalar(1e-9);
};

template <typename Scalar>
struct SubspaceStatus {
    bool valid = false;
    bool degenerate = false;
    Scalar condition_number = std::numeric_limits<Scalar>::quiet_NaN();
    Eigen::Matrix<Scalar, 3, 1> eigenvalues =
        Eigen::Matrix<Scalar, 3, 1>::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    Eigen::Matrix<Scalar, 3, 3> eigenvectors =
        Eigen::Matrix<Scalar, 3, 3>::Identity();
    std::array<bool, 3> degenerate_mask = {{false, false, false}};
    // Columns are Schur eigen-directions; rows are normalized X/Y/Z
    // coefficient magnitudes (the paper's physical-axis contribution ratio).
    Eigen::Matrix<Scalar, 3, 3> physical_axis_share =
        Eigen::Matrix<Scalar, 3, 3>::Zero();
};

template <typename Scalar>
struct Status {
    bool triggered = false;
    bool valid = false;
    bool degenerate = false;
    SubspaceStatus<Scalar> rotation;
    SubspaceStatus<Scalar> translation;
    // FAST-LIO state order is [translation, rotation].
    std::array<bool, 6> degenerate_mask_tr =
        {{false, false, false, false, false, false}};
    // MAP information increment in FAST-LIO order [translation, rotation].
    Eigen::Matrix<Scalar, 6, 6> regularization_tr =
        Eigen::Matrix<Scalar, 6, 6>::Zero();
};

template <typename Scalar>
class Analyzer {
public:
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<Scalar, 6, 6> Matrix6;

    explicit Analyzer(const Config<Scalar> &config) : config_(config) {}

    Status<Scalar> analyzeAndRegularize(Matrix6 &hessian_tr) const {
        Status<Scalar> status;
        status.triggered = true;

        if (!hessian_tr.allFinite()) {
            return status;
        }
        hessian_tr = Scalar(0.5) * (hessian_tr + hessian_tr.transpose());

        // The paper uses [rotation, translation], while FAST-LIO stores
        // [position, rotation] in the first six error-state dimensions.
        Matrix6 permutation = Matrix6::Zero();
        permutation.template block<3, 3>(0, 3).setIdentity();
        permutation.template block<3, 3>(3, 0).setIdentity();
        const Matrix6 hessian_rt = permutation * hessian_tr * permutation.transpose();

        const Matrix3 h_rr = hessian_rt.template block<3, 3>(0, 0);
        const Matrix3 h_rt = hessian_rt.template block<3, 3>(0, 3);
        const Matrix3 h_tr = hessian_rt.template block<3, 3>(3, 0);
        const Matrix3 h_tt = hessian_rt.template block<3, 3>(3, 3);

        Matrix3 h_rr_inverse;
        Matrix3 h_tt_inverse;
        if (!pseudoInverseSymmetric(h_rr, h_rr_inverse) ||
            !pseudoInverseSymmetric(h_tt, h_tt_inverse)) {
            return status;
        }

        // Decoupled observability after optimally eliminating the other
        // variable block. Cross terms are retained in the final Hessian.
        Matrix3 schur_rotation = h_rr - h_rt * h_tt_inverse * h_tr;
        Matrix3 schur_translation = h_tt - h_tr * h_rr_inverse * h_rt;
        schur_rotation = Scalar(0.5) * (schur_rotation + schur_rotation.transpose());
        schur_translation =
            Scalar(0.5) * (schur_translation + schur_translation.transpose());

        if (!analyzeSubspace(schur_rotation, status.rotation) ||
            !analyzeSubspace(schur_translation, status.translation)) {
            return status;
        }

        status.valid = true;
        status.degenerate = status.rotation.degenerate || status.translation.degenerate;
        status.degenerate_mask_tr = {{
            status.translation.degenerate_mask[0],
            status.translation.degenerate_mask[1],
            status.translation.degenerate_mask[2],
            status.rotation.degenerate_mask[0],
            status.rotation.degenerate_mask[1],
            status.rotation.degenerate_mask[2]
        }};

        if (!status.degenerate || config_.regularization_alpha <= Scalar(0)) {
            return status;
        }

        // Theorem 4 in the paper: Gamma = V(tilde Lambda - Lambda)V^T.
        // Adding Gamma clamps only weak Schur directions and yields a MAP
        // regularized system with the requested condition-number bound.
        Matrix6 regularization_rt = Matrix6::Zero();
        regularization_rt.template block<3, 3>(0, 0) =
            buildSubspaceRegularization(status.rotation);
        regularization_rt.template block<3, 3>(3, 3) =
            buildSubspaceRegularization(status.translation);
        status.regularization_tr =
            permutation.transpose() * regularization_rt * permutation;
        hessian_tr += status.regularization_tr;
        hessian_tr = Scalar(0.5) * (hessian_tr + hessian_tr.transpose());
        return status;
    }

private:
    bool pseudoInverseSymmetric(const Matrix3 &matrix, Matrix3 &inverse) const {
        Eigen::SelfAdjointEigenSolver<Matrix3> solver(
            Scalar(0.5) * (matrix + matrix.transpose()));
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
            return false;
        }
        const Scalar max_eigen =
            std::max(solver.eigenvalues().cwiseAbs().maxCoeff(), Scalar(1));
        const Scalar cutoff =
            std::max(config_.inverse_relative_threshold * max_eigen,
                     std::numeric_limits<Scalar>::epsilon());
        Eigen::Matrix<Scalar, 3, 1> inverse_eigenvalues;
        for (int i = 0; i < 3; ++i) {
            const Scalar eigenvalue = solver.eigenvalues()(i);
            inverse_eigenvalues(i) =
                eigenvalue > cutoff ? Scalar(1) / eigenvalue : Scalar(0);
        }
        inverse = solver.eigenvectors() * inverse_eigenvalues.asDiagonal() *
                  solver.eigenvectors().transpose();
        return inverse.allFinite();
    }

    bool analyzeSubspace(const Matrix3 &schur,
                         SubspaceStatus<Scalar> &subspace) const {
        Eigen::SelfAdjointEigenSolver<Matrix3> solver(schur);
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
            return false;
        }

        subspace.valid = true;
        subspace.eigenvalues = solver.eigenvalues().cwiseMax(Scalar(0));
        subspace.eigenvectors = solver.eigenvectors();
        subspace.physical_axis_share = subspace.eigenvectors.cwiseAbs();
        for (int column = 0; column < 3; ++column) {
            const Scalar l1_norm = std::max(
                subspace.physical_axis_share.col(column).sum(),
                std::numeric_limits<Scalar>::epsilon());
            subspace.physical_axis_share.col(column) /= l1_norm;
        }

        const Scalar max_eigen =
            std::max(subspace.eigenvalues.maxCoeff(),
                     std::numeric_limits<Scalar>::epsilon());
        const Scalar min_eigen =
            std::max(subspace.eigenvalues.minCoeff(),
                     std::numeric_limits<Scalar>::epsilon());
        subspace.condition_number = max_eigen / min_eigen;

        for (int i = 0; i < 3; ++i) {
            const Scalar eigenvalue =
                std::max(subspace.eigenvalues(i),
                         std::numeric_limits<Scalar>::epsilon());
            const Scalar directional_condition = max_eigen / eigenvalue;
            subspace.degenerate_mask[i] =
                (config_.eigenvalue_threshold > Scalar(0) &&
                 subspace.eigenvalues(i) < config_.eigenvalue_threshold) ||
                directional_condition > config_.condition_threshold;
            subspace.degenerate =
                subspace.degenerate || subspace.degenerate_mask[i];
        }
        return true;
    }

    Matrix3 buildSubspaceRegularization(
        const SubspaceStatus<Scalar> &subspace) const {
        Eigen::Matrix<Scalar, 3, 1> increments =
            Eigen::Matrix<Scalar, 3, 1>::Zero();
        const Scalar max_eigen =
            std::max(subspace.eigenvalues.maxCoeff(), Scalar(1));
        const Scalar safe_kappa = std::max(config_.kappa_target, Scalar(1));
        const Scalar target =
            std::max(config_.eigenvalue_threshold, max_eigen / safe_kappa);

        for (int i = 0; i < 3; ++i) {
            if (subspace.degenerate_mask[i]) {
                increments(i) = config_.regularization_alpha *
                    std::max(target - subspace.eigenvalues(i), Scalar(0));
            }
        }
        return subspace.eigenvectors * increments.asDiagonal() *
               subspace.eigenvectors.transpose();
    }

    Config<Scalar> config_;
};

}  // namespace dcreg
}  // namespace fast_lio

#endif
