#pragma once

#include <array>

namespace panda_tracker {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;
using Transform = std::array<double, 16>;

Transform identity_transform();

Transform multiply_transform(
    const Transform& left,
    const Transform& right);

Transform invert_transform(
    const Transform& transform);

Transform franka_column_major_transform(
    const std::array<double, 16>& column_major);

Vector3 transform_translation(
    const Transform& transform);

Matrix3 transform_rotation(
    const Transform& transform);

Vector3 so3_log(
    const Matrix3& rotation);

double vector_norm(
    const Vector3& vector);

Vector3 clamp_norm(
    const Vector3& vector,
    double maximum_norm);

bool finite_rigid_transform(
    const Transform& transform,
    double tolerance = 1e-6);

}  // namespace panda_tracker
