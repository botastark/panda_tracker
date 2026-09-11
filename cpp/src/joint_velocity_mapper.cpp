#include "panda_tracker/joint_velocity_mapper.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace panda_tracker {
namespace {

using Matrix3 = std::array<double, 9>;

double& M(Matrix3& a, std::size_t row, std::size_t col) {
  return a[row * 3 + col];
}


double J(
    const Jacobian6x7& jacobian,
    std::size_t row,
    std::size_t col) {
  // libfranka arrays map directly to Eigen's default column-major 6x7 matrix.
  return jacobian[col * 6 + row];
}

bool finite3(const Vector3Velocity& values) {
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool finite7(const JointVector7& values) {
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool solve_3x3(
    Matrix3 A,
    Vector3Velocity b,
    Vector3Velocity& x) {
  // Gaussian elimination with partial pivoting.
  for (std::size_t col = 0; col < 3; ++col) {
    std::size_t pivot = col;
    double pivot_abs = std::abs(M(A, pivot, col));

    for (std::size_t row = col + 1; row < 3; ++row) {
      const double candidate = std::abs(M(A, row, col));
      if (candidate > pivot_abs) {
        pivot = row;
        pivot_abs = candidate;
      }
    }

    if (!(pivot_abs > 1e-14) || !std::isfinite(pivot_abs)) {
      return false;
    }

    if (pivot != col) {
      for (std::size_t k = col; k < 3; ++k) {
        std::swap(M(A, col, k), M(A, pivot, k));
      }
      std::swap(b[col], b[pivot]);
    }

    const double diagonal = M(A, col, col);
    for (std::size_t row = col + 1; row < 3; ++row) {
      const double factor = M(A, row, col) / diagonal;
      M(A, row, col) = 0.0;
      for (std::size_t k = col + 1; k < 3; ++k) {
        M(A, row, k) -= factor * M(A, col, k);
      }
      b[row] -= factor * b[col];
    }
  }

  for (std::size_t reverse = 0; reverse < 3; ++reverse) {
    const std::size_t row = 2 - reverse;
    double rhs = b[row];

    for (std::size_t col = row + 1; col < 3; ++col) {
      rhs -= M(A, row, col) * x[col];
    }

    const double diagonal = M(A, row, row);
    if (!(std::abs(diagonal) > 1e-14) || !std::isfinite(diagonal)) {
      return false;
    }
    x[row] = rhs / diagonal;
  }

  return finite3(x);
}

}  // namespace

Twist6 jacobian_times_joint_velocity(
    const Jacobian6x7& jacobian,
    const JointVector7& qdot) {
  Twist6 twist{};

  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t joint = 0; joint < 7; ++joint) {
      twist[row] += J(jacobian, row, joint) * qdot[joint];
    }
  }

  return twist;
}

JointVelocityMappingResult map_translation_to_joint_velocity(
    const Jacobian6x7& jacobian,
    const Vector3Velocity& desired_velocity,
    double damping) {
  JointVelocityMappingResult result{};

  if (!(damping > 0.0) || !std::isfinite(damping) ||
      !finite3(desired_velocity)) {
    return result;
  }

  for (double value : jacobian) {
    if (!std::isfinite(value)) return result;
  }

  // A = Jv Jv^T + lambda^2 I, where Jv is the first 3 rows.
  Matrix3 A{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      double value = 0.0;
      for (std::size_t joint = 0; joint < 7; ++joint) {
        value +=
            J(jacobian, row, joint) *
            J(jacobian, col, joint);
      }

      if (row == col) {
        value += damping * damping;
      }

      M(A, row, col) = value;
    }
  }

  Vector3Velocity solved{};
  if (!solve_3x3(A, desired_velocity, solved)) {
    return result;
  }

  // qdot = Jv^T solved.
  for (std::size_t joint = 0; joint < 7; ++joint) {
    for (std::size_t row = 0; row < 3; ++row) {
      result.qdot_radps[joint] +=
          J(jacobian, row, joint) * solved[row];
    }
  }

  if (!finite7(result.qdot_radps)) {
    result.qdot_radps = {};
    return result;
  }

  result.achieved_twist =
      jacobian_times_joint_velocity(
          jacobian, result.qdot_radps);

  for (double value : result.achieved_twist) {
    if (!std::isfinite(value)) {
      result.qdot_radps = {};
      result.achieved_twist = {};
      return result;
    }
  }

  result.valid = true;
  return result;
}

double max_abs_joint_value(const JointVector7& values) {
  double result = 0.0;
  for (double value : values) {
    result = std::max(result, std::abs(value));
  }
  return result;
}

}  // namespace panda_tracker
