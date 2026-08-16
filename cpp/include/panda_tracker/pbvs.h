#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace panda_tracker {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;
using Transform = std::array<double, 16>;

struct PbvsConfig {
  double control_rate_hz{100.0};
  bool control_orientation{true};
  double kp_position{1.0};
  double kp_orientation{0.5};
  double max_linear_speed{0.04};
  double max_angular_speed{0.0};
  double max_command_lead{0.005};
  double panda_state_timeout{0.1};
  double tracker_timeout{0.1};
  double max_tracker_position_jump{0.05};
  double max_tracker_angle_jump{0.0};
  double max_enable_position_error{0.15};
  double max_enable_orientation_error{0.0};
  std::size_t consecutive_valid_required{20};
  bool target_feedforward_enabled{false};
  double target_velocity_filter_alpha{0.25};
  double max_target_linear_speed{0.03};
  double max_target_angular_speed{0.0};
  Vector3 workspace_min{{-1.0, -1.0, -1.0}};
  Vector3 workspace_max{{1.0, 1.0, 1.0}};
  Transform T_ES{};
  Transform T_TS_des{};
  std::string tool_geometry_status{};
};

enum class PbvsState {
  kWaitForRobot,
  kWaitForTaskPose,
  kReady,
  kTracking,
  kHold,
  kFault,
};

struct TaskPoseMeasurement {
  Transform T_TS{};
  double timestamp_s{0.0};
  bool valid{false};
  std::uint64_t sequence_id{0};
};

struct PbvsResult {
  PbvsState state{PbvsState::kWaitForRobot};
  std::string reason{};
  Vector3 position_error_base_m{};
  Vector3 orientation_error_body_rad{};
  Vector3 proposed_linear_velocity_base_mps{};
  Vector3 proposed_angular_velocity_body_radps{};
  double position_error_norm_m{0.0};
  double orientation_error_norm_rad{0.0};
  double target_linear_speed_mps{0.0};
  double target_angular_speed_radps{0.0};
  double proposed_linear_speed_mps{0.0};
  double proposed_angular_speed_radps{0.0};
  double proposed_command_lead_m{0.0};
  bool has_proposed_pose{false};
  Transform proposed_T_BE{};
};

Transform identity_transform();
Transform multiply_transform(const Transform& left, const Transform& right);
Transform invert_transform(const Transform& transform);
Transform franka_column_major_transform(
    const std::array<double, 16>& column_major);
Vector3 transform_translation(const Transform& transform);
Matrix3 transform_rotation(const Transform& transform);
Vector3 so3_log(const Matrix3& rotation);
Matrix3 so3_exp(const Vector3& rotation_vector);
double vector_norm(const Vector3& vector);
Vector3 clamp_norm(const Vector3& vector, double maximum_norm);
bool finite_rigid_transform(
    const Transform& transform,
    double tolerance = 1e-6);

bool validate_pbvs_config(const PbvsConfig& config, std::string& error);
bool load_pbvs_config(
    const std::string& path,
    PbvsConfig& config,
    std::string& error);

const char* pbvs_state_text(PbvsState state);

class PbvsController {
 public:
  explicit PbvsController(PbvsConfig config);

  PbvsResult step(
      const std::optional<Transform>& T_BE,
      double robot_state_age_s,
      const std::optional<TaskPoseMeasurement>& task_pose,
      double now_s,
      double dt_s);

  const PbvsConfig& config() const { return config_; }
  std::size_t valid_measurement_count() const { return valid_count_; }

 private:
  bool task_pose_valid(
      const TaskPoseMeasurement& measurement,
      double now_s) const;
  Transform goal_pose(
      const Transform& T_BE,
      const Transform& T_TS) const;
  void reset_target_velocity_estimator();
  void update_target_velocity_estimate(
      const Transform& goal,
      double measurement_timestamp_s);
  PbvsResult hold_result(
      PbvsState state,
      const std::string& reason,
      const Transform& T_BE) const;

  PbvsConfig config_{};
  PbvsState state_{PbvsState::kWaitForRobot};
  std::optional<TaskPoseMeasurement> last_task_pose_{};
  std::optional<std::uint64_t> last_processed_task_sequence_{};
  std::size_t valid_count_{0};
  std::optional<Transform> proposed_pose_{};
  std::optional<Transform> last_goal_pose_{};
  std::optional<double> last_goal_timestamp_s_{};
  Vector3 target_linear_velocity_{};
  Vector3 target_angular_velocity_{};
};

}  // namespace panda_tracker
