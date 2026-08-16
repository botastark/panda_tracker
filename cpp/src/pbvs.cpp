#include "panda_tracker/pbvs.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace panda_tracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double& matrix_at(Matrix3& matrix, std::size_t row, std::size_t column) {
  return matrix[row * 3 + column];
}

double matrix_at(
    const Matrix3& matrix,
    std::size_t row,
    std::size_t column) {
  return matrix[row * 3 + column];
}

Matrix3 identity_matrix3() {
  return {{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
}

Matrix3 transpose(const Matrix3& matrix) {
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      matrix_at(result, row, column) = matrix_at(matrix, column, row);
    }
  }
  return result;
}

Matrix3 multiply(const Matrix3& left, const Matrix3& right) {
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      double value = 0.0;
      for (std::size_t inner = 0; inner < 3; ++inner) {
        value += matrix_at(left, row, inner) *
                 matrix_at(right, inner, column);
      }
      matrix_at(result, row, column) = value;
    }
  }
  return result;
}

Vector3 multiply(const Matrix3& matrix, const Vector3& vector) {
  Vector3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row] += matrix_at(matrix, row, column) * vector[column];
    }
  }
  return result;
}

Vector3 add(const Vector3& left, const Vector3& right) {
  return {{
      left[0] + right[0],
      left[1] + right[1],
      left[2] + right[2],
  }};
}

Vector3 subtract(const Vector3& left, const Vector3& right) {
  return {{
      left[0] - right[0],
      left[1] - right[1],
      left[2] - right[2],
  }};
}

Vector3 scale(const Vector3& vector, double factor) {
  return {{
      factor * vector[0],
      factor * vector[1],
      factor * vector[2],
  }};
}

Matrix3 skew(const Vector3& vector) {
  return {{
      0.0, -vector[2], vector[1],
      vector[2], 0.0, -vector[0],
      -vector[1], vector[0], 0.0,
  }};
}

Transform make_transform(
    const Matrix3& rotation,
    const Vector3& translation) {
  Transform result = identity_transform();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row * 4 + column] = matrix_at(rotation, row, column);
    }
    result[row * 4 + 3] = translation[row];
  }
  return result;
}

bool finite(double value) {
  return std::isfinite(value);
}

PbvsResult base_result(PbvsState state, const std::string& reason) {
  PbvsResult result{};
  result.state = state;
  result.reason = reason;
  return result;
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
    for (std::size_t column = 0; column < 4; ++column) {
      double value = 0.0;
      for (std::size_t inner = 0; inner < 4; ++inner) {
        value += left[row * 4 + inner] * right[inner * 4 + column];
      }
      result[row * 4 + column] = value;
    }
  }
  return result;
}

Transform invert_transform(const Transform& transform) {
  const Matrix3 rotation = transform_rotation(transform);
  const Matrix3 rotation_transpose = transpose(rotation);
  const Vector3 translation = transform_translation(transform);
  return make_transform(
      rotation_transpose,
      scale(multiply(rotation_transpose, translation), -1.0));
}

Transform franka_column_major_transform(
    const std::array<double, 16>& column_major) {
  Transform row_major{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      row_major[row * 4 + column] = column_major[column * 4 + row];
    }
  }
  return row_major;
}

Vector3 transform_translation(const Transform& transform) {
  return {{transform[3], transform[7], transform[11]}};
}

Matrix3 transform_rotation(const Transform& transform) {
  return {{
      transform[0], transform[1], transform[2],
      transform[4], transform[5], transform[6],
      transform[8], transform[9], transform[10],
  }};
}

double vector_norm(const Vector3& vector) {
  return std::sqrt(
      vector[0] * vector[0] +
      vector[1] * vector[1] +
      vector[2] * vector[2]);
}

Vector3 clamp_norm(const Vector3& vector, double maximum_norm) {
  const double norm = vector_norm(vector);
  if (norm <= maximum_norm || norm < 1e-12) {
    return vector;
  }
  return scale(vector, maximum_norm / norm);
}

Vector3 so3_log(const Matrix3& rotation) {
  const double cos_theta = std::clamp(
      (rotation[0] + rotation[4] + rotation[8] - 1.0) * 0.5,
      -1.0,
      1.0);
  const double theta = std::acos(cos_theta);

  const Vector3 antisymmetric{{
      rotation[7] - rotation[5],
      rotation[2] - rotation[6],
      rotation[3] - rotation[1],
  }};

  if (theta < 1e-8) {
    return scale(antisymmetric, 0.5);
  }

  if (kPi - theta < 1e-6) {
    Vector3 axis{{
        std::sqrt(std::max(0.0, (rotation[0] + 1.0) * 0.5)),
        std::sqrt(std::max(0.0, (rotation[4] + 1.0) * 0.5)),
        std::sqrt(std::max(0.0, (rotation[8] + 1.0) * 0.5)),
    }};

    std::size_t largest = 0;
    if (axis[1] > axis[largest]) {
      largest = 1;
    }
    if (axis[2] > axis[largest]) {
      largest = 2;
    }

    if (axis[largest] < 1e-8) {
      return {{theta, 0.0, 0.0}};
    }

    if (largest == 0) {
      axis[1] = (rotation[1] + rotation[3]) / (4.0 * axis[0]);
      axis[2] = (rotation[2] + rotation[6]) / (4.0 * axis[0]);
    } else if (largest == 1) {
      axis[0] = (rotation[1] + rotation[3]) / (4.0 * axis[1]);
      axis[2] = (rotation[5] + rotation[7]) / (4.0 * axis[1]);
    } else {
      axis[0] = (rotation[2] + rotation[6]) / (4.0 * axis[2]);
      axis[1] = (rotation[5] + rotation[7]) / (4.0 * axis[2]);
    }

    const double axis_norm = vector_norm(axis);
    return axis_norm > 1e-12
        ? scale(axis, theta / axis_norm)
        : Vector3{{theta, 0.0, 0.0}};
  }

  return scale(antisymmetric, theta / (2.0 * std::sin(theta)));
}

Matrix3 so3_exp(const Vector3& rotation_vector) {
  const double theta = vector_norm(rotation_vector);
  const Matrix3 identity = identity_matrix3();
  if (theta < 1e-10) {
    const Matrix3 rotation_hat = skew(rotation_vector);
    Matrix3 result = identity;
    for (std::size_t index = 0; index < result.size(); ++index) {
      result[index] += rotation_hat[index];
    }
    return result;
  }

  const Vector3 axis = scale(rotation_vector, 1.0 / theta);
  const Matrix3 axis_hat = skew(axis);
  const Matrix3 axis_hat_squared = multiply(axis_hat, axis_hat);
  Matrix3 result = identity;
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] += std::sin(theta) * axis_hat[index] +
                     (1.0 - std::cos(theta)) * axis_hat_squared[index];
  }
  return result;
}

bool finite_rigid_transform(
    const Transform& transform,
    double tolerance) {
  if (!(tolerance > 0.0) || !finite(tolerance)) {
    return false;
  }
  for (const double value : transform) {
    if (!finite(value)) {
      return false;
    }
  }
  if (std::abs(transform[12]) > tolerance ||
      std::abs(transform[13]) > tolerance ||
      std::abs(transform[14]) > tolerance ||
      std::abs(transform[15] - 1.0) > tolerance) {
    return false;
  }

  const Matrix3 rotation = transform_rotation(transform);
  const Matrix3 product = multiply(transpose(rotation), rotation);
  const Matrix3 identity = identity_matrix3();
  for (std::size_t index = 0; index < product.size(); ++index) {
    if (std::abs(product[index] - identity[index]) > tolerance) {
      return false;
    }
  }

  const double determinant =
      rotation[0] * (rotation[4] * rotation[8] -
                     rotation[5] * rotation[7]) -
      rotation[1] * (rotation[3] * rotation[8] -
                     rotation[5] * rotation[6]) +
      rotation[2] * (rotation[3] * rotation[7] -
                     rotation[4] * rotation[6]);
  return std::abs(determinant - 1.0) <= tolerance;
}

const char* pbvs_state_text(PbvsState state) {
  switch (state) {
    case PbvsState::kWaitForRobot:
      return "WAIT_FOR_ROBOT";
    case PbvsState::kWaitForTaskPose:
      return "WAIT_FOR_TASK_POSE";
    case PbvsState::kReady:
      return "READY";
    case PbvsState::kTracking:
      return "TRACKING";
    case PbvsState::kHold:
      return "HOLD";
    case PbvsState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

bool validate_pbvs_config(const PbvsConfig& config, std::string& error) {
  auto positive = [&](double value, const char* name) {
    if (!finite(value) || !(value > 0.0)) {
      error = std::string(name) + " must be finite and positive.";
      return false;
    }
    return true;
  };
  auto nonnegative = [&](double value, const char* name) {
    if (!finite(value) || value < 0.0) {
      error = std::string(name) + " must be finite and non-negative.";
      return false;
    }
    return true;
  };

  if (!positive(config.control_rate_hz, "control_rate_hz") ||
      !nonnegative(config.kp_position, "kp_position") ||
      !nonnegative(config.kp_orientation, "kp_orientation") ||
      !positive(config.max_linear_speed, "max_linear_speed") ||
      !positive(config.max_angular_speed, "max_angular_speed") ||
      !positive(config.max_command_lead, "max_command_lead") ||
      !positive(config.panda_state_timeout, "panda_state_timeout") ||
      !positive(config.tracker_timeout, "tracker_timeout") ||
      !positive(
          config.max_tracker_position_jump,
          "max_tracker_position_jump") ||
      !positive(config.max_tracker_angle_jump, "max_tracker_angle_jump") ||
      !positive(
          config.max_enable_position_error,
          "max_enable_position_error") ||
      !positive(
          config.max_enable_orientation_error,
          "max_enable_orientation_error") ||
      !nonnegative(
          config.max_target_linear_speed,
          "max_target_linear_speed") ||
      !nonnegative(
          config.max_target_angular_speed,
          "max_target_angular_speed")) {
    return false;
  }
  if (config.control_rate_hz > 1000.0) {
    error = "control_rate_hz must not exceed 1000.";
    return false;
  }
  if (config.consecutive_valid_required == 0) {
    error = "consecutive_valid_required must be positive.";
    return false;
  }
  if (!finite(config.target_velocity_filter_alpha) ||
      !(config.target_velocity_filter_alpha > 0.0) ||
      config.target_velocity_filter_alpha > 1.0) {
    error = "target_velocity_filter_alpha must be in (0, 1].";
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!finite(config.workspace_min[axis]) ||
        !finite(config.workspace_max[axis]) ||
        config.workspace_min[axis] > config.workspace_max[axis]) {
      error = "workspace bounds are invalid.";
      return false;
    }
  }
  if (!finite_rigid_transform(config.T_ES)) {
    error = "T_ES is not a finite rigid transform.";
    return false;
  }
  if (!finite_rigid_transform(config.T_TS_des)) {
    error = "T_TS_des is not a finite rigid transform.";
    return false;
  }
  error.clear();
  return true;
}

PbvsController::PbvsController(PbvsConfig config)
    : config_(std::move(config)) {
  std::string error;
  if (!validate_pbvs_config(config_, error)) {
    throw std::invalid_argument("Invalid PBVS config: " + error);
  }
}

bool PbvsController::task_pose_valid(
    const TaskPoseMeasurement& measurement,
    double now_s) const {
  if (!measurement.valid || !finite_rigid_transform(measurement.T_TS)) {
    return false;
  }

  const double age = now_s - measurement.timestamp_s;
  if (!finite(age) || age < 0.0 || age > config_.tracker_timeout) {
    return false;
  }

  if (last_task_pose_) {
    const Transform delta = multiply_transform(
        invert_transform(last_task_pose_->T_TS),
        measurement.T_TS);
    if (vector_norm(transform_translation(delta)) >
        config_.max_tracker_position_jump) {
      return false;
    }
    if (config_.control_orientation &&
        vector_norm(so3_log(transform_rotation(delta))) >
            config_.max_tracker_angle_jump) {
      return false;
    }
  }
  return true;
}

Transform PbvsController::goal_pose(
    const Transform& T_BE,
    const Transform& T_TS) const {
  const Transform delta_T_S = multiply_transform(
      invert_transform(T_TS),
      config_.T_TS_des);
  const Transform delta_T_E = multiply_transform(
      multiply_transform(config_.T_ES, delta_T_S),
      invert_transform(config_.T_ES));
  return multiply_transform(T_BE, delta_T_E);
}

void PbvsController::reset_target_velocity_estimator() {
  last_goal_pose_.reset();
  last_goal_timestamp_s_.reset();
  target_linear_velocity_ = Vector3{};
  target_angular_velocity_ = Vector3{};
}

void PbvsController::update_target_velocity_estimate(
    const Transform& goal,
    double measurement_timestamp_s) {
  if (!last_goal_pose_ || !last_goal_timestamp_s_) {
    last_goal_pose_ = goal;
    last_goal_timestamp_s_ = measurement_timestamp_s;
    target_linear_velocity_ = Vector3{};
    target_angular_velocity_ = Vector3{};
    return;
  }

  const double sample_dt =
      measurement_timestamp_s - *last_goal_timestamp_s_;
  if (sample_dt <= 1e-6 || sample_dt > config_.tracker_timeout) {
    last_goal_pose_ = goal;
    last_goal_timestamp_s_ = measurement_timestamp_s;
    target_linear_velocity_ = Vector3{};
    target_angular_velocity_ = Vector3{};
    return;
  }

  Vector3 raw_linear_velocity = scale(
      subtract(
          transform_translation(goal),
          transform_translation(*last_goal_pose_)),
      1.0 / sample_dt);
  raw_linear_velocity = clamp_norm(
      raw_linear_velocity,
      config_.max_target_linear_speed);

  const Matrix3 relative_rotation = multiply(
      transpose(transform_rotation(*last_goal_pose_)),
      transform_rotation(goal));
  Vector3 raw_angular_velocity = scale(
      so3_log(relative_rotation),
      1.0 / sample_dt);
  raw_angular_velocity = clamp_norm(
      raw_angular_velocity,
      config_.max_target_angular_speed);

  const double alpha = config_.target_velocity_filter_alpha;
  target_linear_velocity_ = add(
      scale(target_linear_velocity_, 1.0 - alpha),
      scale(raw_linear_velocity, alpha));
  target_angular_velocity_ = add(
      scale(target_angular_velocity_, 1.0 - alpha),
      scale(raw_angular_velocity, alpha));

  last_goal_pose_ = goal;
  last_goal_timestamp_s_ = measurement_timestamp_s;
}

PbvsResult PbvsController::hold_result(
    PbvsState state,
    const std::string& reason,
    const Transform& T_BE) const {
  PbvsResult result = base_result(state, reason);
  result.has_proposed_pose = true;
  result.proposed_T_BE = T_BE;
  return result;
}

PbvsResult PbvsController::step(
    const std::optional<Transform>& T_BE,
    double robot_state_age_s,
    const std::optional<TaskPoseMeasurement>& task_pose,
    double now_s,
    double dt_s) {
  if (!finite(now_s) || !finite(dt_s) || !(dt_s > 0.0)) {
    state_ = PbvsState::kFault;
    proposed_pose_.reset();
    valid_count_ = 0;
    reset_target_velocity_estimator();
    return base_result(state_, "invalid_controller_time");
  }

  if (!T_BE || !finite_rigid_transform(*T_BE) ||
      !finite(robot_state_age_s) || robot_state_age_s < 0.0 ||
      robot_state_age_s > config_.panda_state_timeout) {
    proposed_pose_.reset();
    state_ = PbvsState::kWaitForRobot;
    valid_count_ = 0;
    last_processed_task_sequence_.reset();
    reset_target_velocity_estimator();
    return base_result(state_, "robot_state_missing_or_stale");
  }

  if (!task_pose) {
    proposed_pose_.reset();
    last_task_pose_.reset();
    last_processed_task_sequence_.reset();
    state_ = PbvsState::kWaitForTaskPose;
    valid_count_ = 0;
    reset_target_velocity_estimator();
    return hold_result(state_, "task_pose_missing", *T_BE);
  }

  const double tracker_age_s = now_s - task_pose->timestamp_s;
  if (!finite(tracker_age_s) || tracker_age_s < 0.0 ||
      tracker_age_s > config_.tracker_timeout) {
    proposed_pose_.reset();
    last_task_pose_.reset();
    last_processed_task_sequence_.reset();
    state_ = PbvsState::kHold;
    valid_count_ = 0;
    reset_target_velocity_estimator();
    return hold_result(state_, "task_pose_stale", *T_BE);
  }

  if (!task_pose_valid(*task_pose, now_s)) {
    proposed_pose_.reset();
    last_processed_task_sequence_.reset();
    state_ = PbvsState::kHold;
    valid_count_ = 0;
    reset_target_velocity_estimator();
    return hold_result(state_, "task_pose_invalid_or_jump", *T_BE);
  }

  Transform goal = goal_pose(*T_BE, task_pose->T_TS);
  const Vector3 position_error = subtract(
      transform_translation(goal),
      transform_translation(*T_BE));
  Vector3 orientation_error{};

  if (config_.control_orientation) {
    const Matrix3 relative_rotation = multiply(
        transpose(transform_rotation(*T_BE)),
        transform_rotation(goal));
    orientation_error = so3_log(relative_rotation);
  } else {
    const Matrix3 current_rotation = transform_rotation(*T_BE);
    const Vector3 goal_translation = transform_translation(goal);
    goal = make_transform(current_rotation, goal_translation);
  }

  const double position_error_norm = vector_norm(position_error);
  const double orientation_error_norm = vector_norm(orientation_error);
  if (position_error_norm > config_.max_enable_position_error ||
      (config_.control_orientation &&
       orientation_error_norm > config_.max_enable_orientation_error)) {
    proposed_pose_.reset();
    state_ = PbvsState::kHold;
    valid_count_ = 0;
    reset_target_velocity_estimator();
    PbvsResult result = hold_result(
        state_, "error_exceeds_enable_threshold", *T_BE);
    result.position_error_base_m = position_error;
    result.orientation_error_body_rad = orientation_error;
    result.position_error_norm_m = position_error_norm;
    result.orientation_error_norm_rad = orientation_error_norm;
    return result;
  }

  const bool new_measurement =
      !last_processed_task_sequence_ ||
      *last_processed_task_sequence_ != task_pose->sequence_id;
  if (new_measurement) {
    last_task_pose_ = *task_pose;
    last_processed_task_sequence_ = task_pose->sequence_id;
    ++valid_count_;
    update_target_velocity_estimate(goal, task_pose->timestamp_s);
  }

  if (valid_count_ < config_.consecutive_valid_required) {
    proposed_pose_.reset();
    state_ = PbvsState::kReady;
    PbvsResult result = hold_result(
        state_, "waiting_for_consecutive_valid_measurements", *T_BE);
    result.position_error_base_m = position_error;
    result.orientation_error_body_rad = orientation_error;
    result.position_error_norm_m = position_error_norm;
    result.orientation_error_norm_rad = orientation_error_norm;
    return result;
  }

  const Vector3 target_linear_velocity =
      config_.target_feedforward_enabled
      ? target_linear_velocity_
      : Vector3{};
  const Vector3 target_angular_velocity =
      config_.target_feedforward_enabled && config_.control_orientation
      ? target_angular_velocity_
      : Vector3{};

  const Vector3 linear_velocity = clamp_norm(
      add(
          target_linear_velocity,
          scale(position_error, config_.kp_position)),
      config_.max_linear_speed);
  const Vector3 angular_velocity = config_.control_orientation
      ? clamp_norm(
            add(
                target_angular_velocity,
                scale(orientation_error, config_.kp_orientation)),
            config_.max_angular_speed)
      : Vector3{};

  if (!proposed_pose_) {
    proposed_pose_ = *T_BE;
  }

  Vector3 proposed_position = add(
      transform_translation(*proposed_pose_),
      scale(linear_velocity, dt_s));
  Vector3 command_lead = subtract(
      proposed_position,
      transform_translation(*T_BE));
  command_lead = clamp_norm(command_lead, config_.max_command_lead);
  proposed_position = add(
      transform_translation(*T_BE),
      command_lead);

  Matrix3 proposed_rotation = transform_rotation(*T_BE);
  if (config_.control_orientation) {
    proposed_rotation = multiply(
        transform_rotation(*proposed_pose_),
        so3_exp(scale(angular_velocity, dt_s)));
  }

  Transform proposed = make_transform(proposed_rotation, proposed_position);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    proposed[axis * 4 + 3] = std::clamp(
        proposed[axis * 4 + 3],
        config_.workspace_min[axis],
        config_.workspace_max[axis]);
  }
  proposed_pose_ = proposed;
  state_ = PbvsState::kTracking;

  PbvsResult result = base_result(state_, "");
  result.position_error_base_m = position_error;
  result.orientation_error_body_rad = orientation_error;
  result.proposed_linear_velocity_base_mps = linear_velocity;
  result.proposed_angular_velocity_body_radps = angular_velocity;
  result.position_error_norm_m = position_error_norm;
  result.orientation_error_norm_rad = orientation_error_norm;
  result.target_linear_speed_mps = vector_norm(target_linear_velocity_);
  result.target_angular_speed_radps = vector_norm(target_angular_velocity_);
  result.proposed_linear_speed_mps = vector_norm(linear_velocity);
  result.proposed_angular_speed_radps = vector_norm(angular_velocity);
  result.proposed_command_lead_m = vector_norm(command_lead);
  result.has_proposed_pose = true;
  result.proposed_T_BE = proposed;
  return result;
}

}  // namespace panda_tracker
