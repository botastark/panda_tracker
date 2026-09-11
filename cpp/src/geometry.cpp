#include "panda_tracker/geometry.h"

#include <algorithm>
#include <cmath>

namespace panda_tracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp_unit(double x) {
  return std::clamp(x, -1.0, 1.0);
}

}  // namespace

Transform identity_transform() {
  return {{
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0,
  }};
}

Transform multiply_transform(
    const Transform& left,
    const Transform& right) {
  Transform result{};

  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      double value = 0.0;
      for (std::size_t k = 0; k < 4; ++k) {
        value += left[row * 4 + k] * right[k * 4 + col];
      }
      result[row * 4 + col] = value;
    }
  }
  return result;
}

Transform invert_transform(const Transform& T) {
  Transform result = identity_transform();

  // R^-1 = R^T
  result[0] = T[0];
  result[1] = T[4];
  result[2] = T[8];

  result[4] = T[1];
  result[5] = T[5];
  result[6] = T[9];

  result[8] = T[2];
  result[9] = T[6];
  result[10] = T[10];

  const Vector3 p = transform_translation(T);

  result[3] =
      -(result[0] * p[0] + result[1] * p[1] + result[2] * p[2]);
  result[7] =
      -(result[4] * p[0] + result[5] * p[1] + result[6] * p[2]);
  result[11] =
      -(result[8] * p[0] + result[9] * p[1] + result[10] * p[2]);

  return result;
}

Transform franka_column_major_transform(
    const std::array<double, 16>& column_major) {
  Transform row_major{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      row_major[row * 4 + col] = column_major[col * 4 + row];
    }
  }
  return row_major;
}

Vector3 transform_translation(const Transform& T) {
  return {{T[3], T[7], T[11]}};
}

Matrix3 transform_rotation(const Transform& T) {
  return {{
      T[0], T[1], T[2],
      T[4], T[5], T[6],
      T[8], T[9], T[10],
  }};
}

Vector3 so3_log(const Matrix3& R) {
  const double trace = R[0] + R[4] + R[8];
  const double angle = std::acos(clamp_unit((trace - 1.0) * 0.5));

  if (angle < 1e-9) {
    return {{
        0.5 * (R[7] - R[5]),
        0.5 * (R[2] - R[6]),
        0.5 * (R[3] - R[1]),
    }};
  }

  // Close to pi, the ordinary axis formula is poorly conditioned.
  if (kPi - angle < 1e-5) {
    Vector3 axis{{
        std::sqrt(std::max(0.0, (R[0] + 1.0) * 0.5)),
        std::sqrt(std::max(0.0, (R[4] + 1.0) * 0.5)),
        std::sqrt(std::max(0.0, (R[8] + 1.0) * 0.5)),
    }};

    if (R[7] - R[5] < 0.0) axis[0] = -axis[0];
    if (R[2] - R[6] < 0.0) axis[1] = -axis[1];
    if (R[3] - R[1] < 0.0) axis[2] = -axis[2];

    const double n = vector_norm(axis);
    if (n > 1e-12) {
      for (double& v : axis) v = v * angle / n;
      return axis;
    }
  }

  const double scale = angle / (2.0 * std::sin(angle));
  return {{
      scale * (R[7] - R[5]),
      scale * (R[2] - R[6]),
      scale * (R[3] - R[1]),
  }};
}

double vector_norm(const Vector3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vector3 clamp_norm(
    const Vector3& v,
    double maximum_norm) {
  const double n = vector_norm(v);
  if (!(maximum_norm > 0.0) || n <= maximum_norm || n < 1e-15) {
    return v;
  }

  const double scale = maximum_norm / n;
  return {{v[0] * scale, v[1] * scale, v[2] * scale}};
}

bool finite_rigid_transform(
    const Transform& T,
    double tolerance) {
  if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
    return false;
  }

  for (double value : T) {
    if (!std::isfinite(value)) return false;
  }

  if (std::abs(T[12]) > tolerance ||
      std::abs(T[13]) > tolerance ||
      std::abs(T[14]) > tolerance ||
      std::abs(T[15] - 1.0) > tolerance) {
    return false;
  }

  const Matrix3 R = transform_rotation(T);

  // Columns of R must be orthonormal.
  for (std::size_t a = 0; a < 3; ++a) {
    for (std::size_t b = 0; b < 3; ++b) {
      double dot = 0.0;
      for (std::size_t row = 0; row < 3; ++row) {
        dot += R[row * 3 + a] * R[row * 3 + b];
      }
      const double expected = (a == b) ? 1.0 : 0.0;
      if (std::abs(dot - expected) > tolerance) return false;
    }
  }

  const double det =
      R[0] * (R[4] * R[8] - R[5] * R[7]) -
      R[1] * (R[3] * R[8] - R[5] * R[6]) +
      R[2] * (R[3] * R[7] - R[4] * R[6]);

  return std::abs(det - 1.0) <= tolerance;
}

}  // namespace panda_tracker
