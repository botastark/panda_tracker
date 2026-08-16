#include "panda_tracker/pbvs.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void require_near(
    double actual,
    double expected,
    double tolerance,
    const std::string& message) {
  require(
      std::abs(actual - expected) <= tolerance,
      message + " actual=" + std::to_string(actual) +
          " expected=" + std::to_string(expected));
}

panda_tracker::PbvsConfig base_config() {
  panda_tracker::PbvsConfig config{};
  config.control_rate_hz = 100.0;
  config.control_orientation = false;
  config.kp_position = 1.0;
  config.kp_orientation = 1.0;
  config.max_linear_speed = 0.2;
  config.max_angular_speed = 0.5;
  config.max_command_lead = 0.05;
  config.panda_state_timeout = 1.0;
  config.tracker_timeout = 1.0;
  config.max_tracker_position_jump = 0.1;
  config.max_tracker_angle_jump = 0.5;
  config.max_enable_position_error = 0.5;
  config.max_enable_orientation_error = 1.5;
  config.consecutive_valid_required = 1;
  config.target_feedforward_enabled = false;
  config.target_velocity_filter_alpha = 1.0;
  config.max_target_linear_speed = 0.2;
  config.max_target_angular_speed = 0.5;
  config.workspace_min = {{-1.0, -1.0, -1.0}};
  config.workspace_max = {{1.0, 1.0, 1.0}};
  config.T_ES = panda_tracker::identity_transform();
  config.T_TS_des = panda_tracker::identity_transform();
  return config;
}

panda_tracker::TaskPoseMeasurement measurement_for_goal_x(
    double goal_x,
    std::uint64_t sequence,
    double timestamp) {
  panda_tracker::TaskPoseMeasurement measurement{};
  measurement.T_TS = panda_tracker::identity_transform();
  measurement.T_TS[3] = -goal_x;
  measurement.timestamp_s = timestamp;
  measurement.valid = true;
  measurement.sequence_id = sequence;
  return measurement;
}

void test_franka_transform_conversion() {
  const std::array<double, 16> column_major{{
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.1, -0.2, 0.3, 1.0,
  }};
  const auto transform =
      panda_tracker::franka_column_major_transform(column_major);
  require_near(transform[3], 0.1, 1e-12, "Franka x conversion");
  require_near(transform[7], -0.2, 1e-12, "Franka y conversion");
  require_near(transform[11], 0.3, 1e-12, "Franka z conversion");
  require(
      panda_tracker::finite_rigid_transform(transform),
      "converted Franka transform is rigid");
}

void test_so3_pi_rotation() {
  const panda_tracker::Matrix3 rotation_x_pi{{
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0,
  }};
  const auto vector = panda_tracker::so3_log(rotation_x_pi);
  require_near(
      panda_tracker::vector_norm(vector),
      kPi,
      1e-9,
      "pi rotation norm");
  require_near(std::abs(vector[0]), kPi, 1e-9, "pi rotation x axis");
}

void test_goal_sign_and_proportional_velocity() {
  auto config = base_config();
  panda_tracker::PbvsController controller(config);
  const auto result = controller.step(
      panda_tracker::identity_transform(),
      0.0,
      measurement_for_goal_x(0.01, 1, 10.0),
      10.0,
      0.01);

  require(
      result.state == panda_tracker::PbvsState::kTracking,
      "identity goal enters tracking");
  require_near(
      result.position_error_base_m[0],
      0.01,
      1e-12,
      "goal transform sign");
  require_near(
      result.proposed_linear_velocity_base_mps[0],
      0.01,
      1e-12,
      "proportional velocity");
  require_near(
      result.proposed_T_BE[3],
      0.0001,
      1e-12,
      "integrated proposed pose");
}

void test_unique_sequence_gate() {
  auto config = base_config();
  config.consecutive_valid_required = 2;
  panda_tracker::PbvsController controller(config);
  const auto measurement = measurement_for_goal_x(0.0, 10, 20.0);

  const auto first = controller.step(
      panda_tracker::identity_transform(), 0.0, measurement, 20.0, 0.01);
  const auto duplicate = controller.step(
      panda_tracker::identity_transform(), 0.0, measurement, 20.01, 0.01);
  auto next = measurement;
  next.sequence_id = 11;
  next.timestamp_s = 20.02;
  const auto second_unique = controller.step(
      panda_tracker::identity_transform(), 0.0, next, 20.02, 0.01);

  require(first.state == panda_tracker::PbvsState::kReady, "first is READY");
  require(
      duplicate.state == panda_tracker::PbvsState::kReady,
      "duplicate remains READY");
  require(
      controller.valid_measurement_count() == 2,
      "only unique measurements counted");
  require(
      second_unique.state == panda_tracker::PbvsState::kTracking,
      "second unique enters tracking");
}

void test_stale_and_jump_hold() {
  auto config = base_config();
  config.tracker_timeout = 0.1;
  config.max_tracker_position_jump = 0.05;

  panda_tracker::PbvsController stale_controller(config);
  const auto stale = stale_controller.step(
      panda_tracker::identity_transform(),
      0.0,
      measurement_for_goal_x(0.0, 1, 1.0),
      1.2,
      0.01);
  require(stale.state == panda_tracker::PbvsState::kHold, "stale holds");
  require(stale.reason == "task_pose_stale", "stale reason");

  panda_tracker::PbvsController jump_controller(config);
  (void)jump_controller.step(
      panda_tracker::identity_transform(),
      0.0,
      measurement_for_goal_x(0.0, 1, 2.0),
      2.0,
      0.01);
  const auto jump = jump_controller.step(
      panda_tracker::identity_transform(),
      0.0,
      measurement_for_goal_x(0.2, 2, 2.01),
      2.01,
      0.01);
  require(jump.state == panda_tracker::PbvsState::kHold, "jump holds");
  require(
      jump.reason == "task_pose_invalid_or_jump",
      "jump rejection reason");
}

void test_feedforward_parity() {
  auto config = base_config();
  config.kp_position = 0.0;
  config.target_feedforward_enabled = true;
  config.max_target_linear_speed = 0.2;
  panda_tracker::PbvsController controller(config);

  (void)controller.step(
      panda_tracker::identity_transform(),
      0.0,
      measurement_for_goal_x(0.0, 1, 30.0),
      30.0,
      0.01);
  const auto result = controller.step(
      panda_tracker::identity_transform(),
      0.0,
      measurement_for_goal_x(0.01, 2, 30.1),
      30.1,
      0.01);

  require_near(
      result.target_linear_speed_mps,
      0.1,
      1e-9,
      "target velocity estimate");
  require_near(
      result.proposed_linear_speed_mps,
      0.1,
      1e-9,
      "feedforward velocity");
  require_near(
      result.proposed_T_BE[3],
      0.001,
      1e-9,
      "feedforward integration");
}

void test_robot_config_load(const std::string& path) {
  panda_tracker::PbvsConfig config{};
  std::string error;
  require(
      panda_tracker::load_pbvs_config(path, config, error),
      "robot config loads: " + error);
  require_near(config.control_rate_hz, 100.0, 1e-12, "control rate");
  require_near(config.max_linear_speed, 0.04, 1e-12, "linear limit");
  require_near(
      config.max_angular_speed,
      2.0 * kPi / 180.0,
      1e-12,
      "angular limit radians");
  require_near(config.T_ES[11], 0.28375, 1e-12, "T_ES z");
  require_near(config.T_TS_des[5], -1.0, 1e-12, "desired rotation y");
  require(
      config.tool_geometry_status.find("do_not_enable_robot") !=
          std::string::npos,
      "tool geometry warning retained");

  const panda_tracker::Transform measured_T_BE{{
      -0.12831616959138459, 0.98957287904391744,
      -0.065425359625508259, -0.0403,
      0.9904972042423279, 0.12458428395145364,
      -0.058258429265092108, 0.6666,
      -0.049499989995223322, -0.072279134285322053,
      -0.99615534819496765, 0.4244,
      0.0, 0.0, 0.0, 1.0,
  }};
  panda_tracker::TaskPoseMeasurement task{};
  task.T_TS = config.T_TS_des;
  task.T_TS[3] = 0.0064;
  task.timestamp_s = 40.0;
  task.valid = true;
  task.sequence_id = 1;

  panda_tracker::PbvsController controller(config);
  const auto parity = controller.step(
      measured_T_BE, 0.0, task, 40.0, 0.01);
  require(
      parity.state == panda_tracker::PbvsState::kReady,
      "robot config first measurement is READY");
  require_near(
      parity.position_error_base_m[0],
      0.000821223485384895,
      1e-12,
      "Python parity p_error x");
  require_near(
      parity.position_error_base_m[1],
      -0.0063391821071509,
      1e-12,
      "Python parity p_error y");
  require_near(
      parity.position_error_base_m[2],
      0.000316799935969447,
      1e-12,
      "Python parity p_error z");
  require_near(
      parity.position_error_norm_m,
      0.0064,
      1e-12,
      "Python parity p_error norm");
  require_near(
      parity.orientation_error_norm_rad,
      0.0,
      1e-9,
      "Python parity orientation error");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " PBVS_CONFIG.json\n";
    return EXIT_FAILURE;
  }

  test_franka_transform_conversion();
  test_so3_pi_rotation();
  test_goal_sign_and_proportional_velocity();
  test_unique_sequence_gate();
  test_stale_and_jump_hold();
  test_feedforward_parity();
  test_robot_config_load(argv[1]);

  std::cout << "PASS: PBVS math, state, and config tests\n";
  return EXIT_SUCCESS;
}
