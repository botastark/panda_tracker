#pragma once

#include "panda_tracker/geometry.h"

#include <array>
#include <cstddef>
#include <string>

namespace panda_tracker {

struct AxisMask {
  bool x{true};
  bool y{false};
  bool z{false};
};

struct TrackerFilterConfig {
  std::size_t median_window_samples{5};
  double ema_alpha{0.25};
  double max_filtered_jump_m{0.050};
};

struct LoadModelConfig {
  bool enabled{true};
  bool includes_end_effector{true};
  double mass_kg{0.0};
  std::array<double, 3> com_F_m{};
  std::array<double, 9> inertia_at_com_F_kg_m2{};
  double settle_s{1.0};
  double max_preexisting_ee_mass_kg{0.001};
};

struct PositionTrackingConfig {
  bool motion_enabled{false};
  bool realtime_enforced{false};
  bool require_tracker_source_filter{true};
  bool stop_on_tracking_loss{true};
  bool stop_on_robot_contact{true};
  double worker_rate_hz{100.0};

  AxisMask control_axes{};
  double kp_position{1.0};
  double max_linear_speed_mps{0.0005};
  double max_linear_acceleration_mps2{0.002};
  double max_linear_jerk_mps3{0.020};
  double max_position_error_m{0.080};

  // While Z tracking is disabled, actively hold the physical flange at its
  // startup base-frame Z position. When control_z=true this hold is bypassed
  // and PBVS owns Z normally.
  bool hold_z_when_control_disabled{true};
  double kp_z_hold{2.0};
  double max_z_hold_speed_mps{0.005};

  // Diagnostic-only setpoint bias added to the PBVS translational error in
  // Panda base frame B. Default zero = normal PBVS.
  Vector3 diagnostic_position_bias_B_m{{0.0, 0.0, 0.0}};

  double robot_state_timeout_s{0.100};
  double tracker_timeout_s{0.150};

  // A stale tracker sample immediately makes the worker command invalid.
  // After arming, the RT loop commands zero velocity during this grace
  // interval and allows the tracker to recover/re-arm. Persistent loss beyond
  // the grace interval terminates the motion.
  double tracking_loss_grace_s{0.500};

  double command_timeout_s{0.050};
  std::size_t arm_valid_packets{10};
  double max_arm_wait_s{15.0};
  // <= 0 means no runtime timeout; all other safety stops remain active.
  double max_motion_runtime_s{5.0};
  double max_stop_time_s{1.0};

  TrackerFilterConfig filter{};

  Vector3 max_travel_m{{0.010, 0.002, 0.002}};
  double braking_margin_m{0.001};
  double max_rotation_travel_deg{2.0};

  double max_start_joint_speed_radps{0.050};
  double max_running_joint_speed_radps{0.500};

  // Joint-velocity motion-generator limits. These are command-shaping limits,
  // intentionally below max_running_joint_speed_radps.
  double max_command_joint_speed_radps{0.350};
  double max_command_joint_acceleration_radps2{0.500};
  double max_command_joint_jerk_radps3{50.0};
  double jacobian_damping{0.050};
  double stop_joint_velocity_epsilon_radps{0.001};

  std::array<double, 7> max_abs_joint_torque_nm{
      {80, 80, 80, 80, 11, 11, 11}};
  std::array<double, 7> max_abs_external_joint_torque_nm{
      {3, 3, 3, 3, 2, 2, 2}};
  std::array<double, 7> max_abs_joint_torque_rate_nmps{
      {800, 800, 800, 800, 300, 300, 300}};

  bool wrench_bias_enabled{true};
  double wrench_bias_settle_s{1.0};
  std::size_t wrench_bias_samples{100};

  double max_external_force_n{15.0};
  double max_external_torque_nm{3.0};
  double max_raw_external_force_n{20.0};
  double max_raw_external_torque_nm{10.0};

  double torque_speed_derate_ratio{0.80};
  double torque_monitor_grace_s{0.50};
  std::size_t torque_violation_cycles{5};

  double min_control_command_success_rate{0.90};
  double communication_grace_s{2.0};

  double stop_velocity_epsilon_mps{0.00002};
  double stop_acceleration_epsilon_mps2{0.0005};

  LoadModelConfig load{};

  // T_XY = pose of Y expressed in X.
  Transform T_FS{};
  Transform T_CS{};
  Transform T_TS_des{};
};

bool load_position_tracking_config(
    const std::string& path,
    PositionTrackingConfig& config,
    std::string& error);

}  // namespace panda_tracker
