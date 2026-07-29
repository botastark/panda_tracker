#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ncs_common.h"

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {

constexpr const char* ROBOT_IP = "172.16.0.2";

constexpr int COMMAND_PORT = 2600;
constexpr int STATE_PORT = 6200;
constexpr const char* STATE_CLIENT_IP = "172.16.0.1";

// Cartesian workspace in Panda base frame.
constexpr double MIN_X = -0.20;
constexpr double MAX_X = 0.20;
constexpr double MIN_Y = 0.36;
constexpr double MAX_Y = 0.70;
constexpr double MIN_Z = 0.05;
constexpr double MAX_Z = 0.55;

// Keep the commanded equilibrium close to the measured pose.
constexpr double MAX_POSITION_LEAD = 0.10;               // metres
constexpr double MAX_ORIENTATION_LEAD = 5.0 * M_PI / 180.0;

// At approximately 1 kHz, 200 unchanged cycles correspond to about 200 ms.
constexpr std::uint64_t COMMAND_TIMEOUT_CYCLES = 200;

#pragma pack(push, 1)
struct Payload {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};
#pragma pack(pop)

static_assert(sizeof(Payload) == 6 * sizeof(float),
              "Payload must contain exactly six packed floats.");

struct Pose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

std::mutex next_pose_mutex;
std::mutex current_state_mutex;

Pose next_pose;
Pose current_state;

// Only the Franka control thread updates this after initialization.
Pose current_target;

// Incremented whenever a valid UDP command is accepted.
std::atomic<std::uint64_t> command_sequence{0};


double clamp_workspace(double value, char axis) {
  double minimum = 0.0;
  double maximum = 0.0;

  switch (axis) {
    case 'x':
      minimum = MIN_X;
      maximum = MAX_X;
      break;
    case 'y':
      minimum = MIN_Y;
      maximum = MAX_Y;
      break;
    case 'z':
      minimum = MIN_Z;
      maximum = MAX_Z;
      break;
    default:
      return value;
  }

  return std::max(minimum, std::min(maximum, value));
}


double limit_position_lead(double measured, double commanded) {
  return std::max(
      measured - MAX_POSITION_LEAD,
      std::min(measured + MAX_POSITION_LEAD, commanded));
}


Pose pose_from_transform(const std::array<double, 16>& transform) {
  Pose pose{};

  pose.x = transform[12];
  pose.y = transform[13];
  pose.z = transform[14];

  // ZYX convention:
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  pose.roll = std::atan2(transform[6], transform[10]);
  pose.pitch = std::atan2(
      -transform[2],
      std::hypot(transform[0], transform[1]));
  pose.yaw = std::atan2(transform[1], transform[0]);

  return pose;
}


bool payload_is_finite(const Payload& payload) {
  return std::isfinite(payload.x) &&
         std::isfinite(payload.y) &&
         std::isfinite(payload.z) &&
         std::isfinite(payload.roll) &&
         std::isfinite(payload.pitch) &&
         std::isfinite(payload.yaw);
}


void store_next_pose(const Payload& payload) {
  Pose command{};

  command.x = clamp_workspace(payload.x, 'x');
  command.y = clamp_workspace(payload.y, 'y');
  command.z = clamp_workspace(payload.z, 'z');

  command.roll = payload.roll;
  command.pitch = payload.pitch;
  command.yaw = payload.yaw;

  {
    // This is a non-real-time networking thread, so blocking briefly is fine.
    std::lock_guard<std::mutex> lock(next_pose_mutex);
    next_pose = command;
  }

  command_sequence.fetch_add(1, std::memory_order_relaxed);

  static int print_counter = 0;
  if (++print_counter >= 500) {
    std::printf(
        "TARGET: xyz=[%.5f %.5f %.5f], "
        "rpy_deg=[%.2f %.2f %.2f]\n",
        command.x,
        command.y,
        command.z,
        command.roll * 180.0 / M_PI,
        command.pitch * 180.0 / M_PI,
        command.yaw * 180.0 / M_PI);
    print_counter = 0;
  }
}


int receive_commands() {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    perror("socket");
    return 1;
  }

  sockaddr_in server_address{};
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(COMMAND_PORT);
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(
          socket_fd,
          reinterpret_cast<sockaddr*>(&server_address),
          sizeof(server_address)) < 0) {
    perror("bind");
    close(socket_fd);
    return 1;
  }

  Payload received_payload{};
  sockaddr_in client_address{};
  socklen_t client_address_length = sizeof(client_address);

  while (true) {
    const ssize_t received_bytes = recvfrom(
        socket_fd,
        &received_payload,
        sizeof(received_payload),
        0,
        reinterpret_cast<sockaddr*>(&client_address),
        &client_address_length);

    if (received_bytes < 0) {
      if (errno == EINTR) {
        continue;
      }

      perror("recvfrom");
      close(socket_fd);
      return 1;
    }

    if (received_bytes != static_cast<ssize_t>(sizeof(Payload))) {
      std::fprintf(
          stderr,
          "Ignoring UDP command of %zd bytes; expected %zu bytes\n",
          received_bytes,
          sizeof(Payload));
      continue;
    }

    if (!payload_is_finite(received_payload)) {
      std::fprintf(stderr, "Ignoring non-finite UDP command\n");
      continue;
    }

    static int print_counter = 0;
    if (++print_counter >= 500) {
      std::printf(
          "RX command: x=%.6f y=%.6f z=%.6f "
          "roll=%.6f pitch=%.6f yaw=%.6f\n",
          received_payload.x,
          received_payload.y,
          received_payload.z,
          received_payload.roll,
          received_payload.pitch,
          received_payload.yaw);
      print_counter = 0;
    }

    store_next_pose(received_payload);
  }
}


int publish_state() {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    perror("socket");
    return 1;
  }

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(STATE_PORT);

  if (inet_pton(AF_INET, STATE_CLIENT_IP, &destination.sin_addr) != 1) {
    std::fprintf(stderr, "Invalid state destination IP: %s\n", STATE_CLIENT_IP);
    close(socket_fd);
    return 1;
  }

  while (true) {
    Payload packet{};
    bool have_state = false;

    // Never make the Franka callback wait for this thread.
    if (current_state_mutex.try_lock()) {
      packet.x = static_cast<float>(current_state.x);
      packet.y = static_cast<float>(current_state.y);
      packet.z = static_cast<float>(current_state.z);
      packet.roll = static_cast<float>(current_state.roll);
      packet.pitch = static_cast<float>(current_state.pitch);
      packet.yaw = static_cast<float>(current_state.yaw);
      current_state_mutex.unlock();
      have_state = true;
    }

    if (have_state) {
      const ssize_t sent_bytes = sendto(
          socket_fd,
          &packet,
          sizeof(packet),
          0,
          reinterpret_cast<sockaddr*>(&destination),
          sizeof(destination));

      if (sent_bytes < 0) {
        perror("sendto");
      }
    }

    // Approximately 1 kHz. Sending faster mostly duplicates robot states.
    usleep(1000);
  }
}


void close_gripper() {
  franka::Robot robot(ROBOT_IP);
  franka::Gripper gripper(ROBOT_IP);

  gripper.homing();

  constexpr double object_width = 0.0;
  constexpr double gripper_speed = 0.2;
  constexpr double force = 200.0;

  std::cout << "Grasping with " << force << " N\n";

  if (!gripper.grasp(
          object_width,
          gripper_speed,
          force)) {
    std::cout << "Failed to grasp object.\n";
  }
}


void initialize_robot_state() {
  franka::Robot robot(ROBOT_IP);
  const franka::RobotState robot_state = robot.readOnce();
  const Pose initial_pose = pose_from_transform(robot_state.O_T_EE);

  {
    std::lock_guard<std::mutex> lock(next_pose_mutex);
    next_pose = initial_pose;
  }

  {
    std::lock_guard<std::mutex> lock(current_state_mutex);
    current_state = initial_pose;
  }

  current_target = initial_pose;

  std::printf(
      "Initial RPY [deg]: %.3f | %.3f | %.3f\n",
      initial_pose.roll * 180.0 / M_PI,
      initial_pose.pitch * 180.0 / M_PI,
      initial_pose.yaw * 180.0 / M_PI);

  for (std::size_t index = 0; index < robot_state.q.size(); ++index) {
    std::printf(
        "Joint %zu: %.3f\n",
        index + 1,
        robot_state.q[index]);
  }
}


Eigen::Quaterniond limited_orientation_target(
    const Eigen::Quaterniond& measured_orientation,
    Eigen::Quaterniond desired_orientation) {
  Eigen::Quaterniond measured = measured_orientation;

  measured.normalize();
  desired_orientation.normalize();

  if (measured.dot(desired_orientation) < 0.0) {
    desired_orientation.coeffs() *= -1.0;
  }

  const double raw_dot = measured.dot(desired_orientation);
  const double quaternion_dot =
      std::max(-1.0, std::min(1.0, raw_dot));

  const double angular_distance =
      2.0 * std::acos(std::abs(quaternion_dot));

  if (angular_distance > MAX_ORIENTATION_LEAD) {
    const double interpolation =
        MAX_ORIENTATION_LEAD / angular_distance;

    desired_orientation =
        measured.slerp(interpolation, desired_orientation);

    desired_orientation.normalize();
  }

  return desired_orientation;
}


void move_end_effector() {
  std::cout << "Are things ready ???\n";
  std::cin.ignore();
  std::cout << ROBOT_IP << '\n';

  constexpr double translational_stiffness = 150.0;
  constexpr double rotational_stiffness = 25.0;

  Eigen::Matrix<double, 6, 6> stiffness =
      Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Matrix<double, 6, 6> damping =
      Eigen::Matrix<double, 6, 6>::Zero();

  stiffness.topLeftCorner<3, 3>() =
      translational_stiffness * Eigen::Matrix3d::Identity();

  stiffness.bottomRightCorner<3, 3>() =
      rotational_stiffness * Eigen::Matrix3d::Identity();

  damping.topLeftCorner<3, 3>() =
      2.0 * std::sqrt(translational_stiffness)
      * Eigen::Matrix3d::Identity();

  damping.bottomRightCorner<3, 3>() =
      2.0 * std::sqrt(rotational_stiffness)
      * Eigen::Matrix3d::Identity();

  try {
    // franka::Robot robot(ROBOT_IP);
    // setDefaultBehavior(robot);
    franka::Robot robot(ROBOT_IP, franka::RealtimeConfig::kIgnore);

    franka::Model model = robot.loadModel();

    /*
     * Keep the default collision behavior for initial orientation tests.
     * Do not replace it with uniformly high thresholds.
     */

    std::uint64_t last_seen_command_sequence =
        command_sequence.load(std::memory_order_relaxed);

    std::uint64_t cycles_without_command = 0;

    auto impedance_control_callback =
        [&](const franka::RobotState& robot_state,
            franka::Duration /*period*/) -> franka::Torques {
      const Eigen::Affine3d measured_transform(
          Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

      const Eigen::Vector3d measured_position(
          measured_transform.translation());

      Eigen::Quaterniond measured_orientation(
          measured_transform.linear());

      measured_orientation.normalize();

      // Publish current measured state without ever blocking this callback.
      if (current_state_mutex.try_lock()) {
        current_state = pose_from_transform(robot_state.O_T_EE);
        current_state_mutex.unlock();
      }

      Pose latest_target = current_target;

      if (next_pose_mutex.try_lock()) {
        latest_target = next_pose;
        next_pose_mutex.unlock();
      }

      const std::uint64_t latest_sequence =
          command_sequence.load(std::memory_order_relaxed);

      if (latest_sequence != last_seen_command_sequence) {
        last_seen_command_sequence = latest_sequence;
        cycles_without_command = 0;
      } else if (cycles_without_command < COMMAND_TIMEOUT_CYCLES + 1) {
        ++cycles_without_command;
      }

      const bool command_stale =
          cycles_without_command > COMMAND_TIMEOUT_CYCLES;

      Eigen::Vector3d desired_position;
      Eigen::Quaterniond desired_orientation;

      if (command_stale) {
        // A stale command stream becomes a compliant hold at the current pose.
        current_target = pose_from_transform(robot_state.O_T_EE);
        desired_position = measured_position;
        desired_orientation = measured_orientation;
      } else {
        current_target.x = limit_position_lead(
            measured_position.x(),
            latest_target.x);

        current_target.y = limit_position_lead(
            measured_position.y(),
            latest_target.y);

        current_target.z = limit_position_lead(
            measured_position.z(),
            latest_target.z);

        current_target.roll = latest_target.roll;
        current_target.pitch = latest_target.pitch;
        current_target.yaw = latest_target.yaw;

        desired_position = Eigen::Vector3d(
            current_target.x,
            current_target.y,
            current_target.z);

        desired_orientation =
            Eigen::AngleAxisd(
                current_target.yaw,
                Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(
                current_target.pitch,
                Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(
                current_target.roll,
                Eigen::Vector3d::UnitX());

        desired_orientation = limited_orientation_target(
            measured_orientation,
            desired_orientation);
      }

      const std::array<double, 7> coriolis_array =
          model.coriolis(robot_state);

      const std::array<double, 42> jacobian_array =
          model.zeroJacobian(
              franka::Frame::kEndEffector,
              robot_state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
          coriolis_array.data());

      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
          jacobian_array.data());

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(
          robot_state.dq.data());

      Eigen::Matrix<double, 6, 1> error;
      error.head<3>() = measured_position - desired_position;

      /*
       * Use the shortest quaternion representation.
       * The following form matches the standard libfranka Cartesian impedance
       * controller convention.
       */
      if (desired_orientation.coeffs().dot(
              measured_orientation.coeffs()) < 0.0) {
        measured_orientation.coeffs() *= -1.0;
      }

      const Eigen::Quaterniond error_quaternion(
          measured_orientation.inverse() * desired_orientation);

      error.tail<3>() <<
          error_quaternion.x(),
          error_quaternion.y(),
          error_quaternion.z();

      error.tail<3>() =
          -measured_transform.linear() * error.tail<3>();

      const Eigen::Matrix<double, 7, 1> tau_task =
          jacobian.transpose()
          * (
              -stiffness * error
              - damping * (jacobian * dq)
          );

      const Eigen::Matrix<double, 7, 1> tau_desired =
          tau_task + coriolis;

      std::array<double, 7> torque_command{};

      Eigen::Map<Eigen::Matrix<double, 7, 1>>(
          torque_command.data()) = tau_desired;

      return torque_command;
    };

    robot.control(impedance_control_callback);

  } catch (const franka::Exception& exception) {
    std::cerr << exception.what() << '\n';
  }
}

}  // namespace


int main(int argc, char** argv) {
  std::cout << "Hello NCS peoplex :)\n";

  if (argc != 3) {
    std::cerr
        << "Usage: " << argv[0]
        << " <operating-mode> <gripper-open-close>\n";
    return EXIT_FAILURE;
  }

  const int operating_mode = std::stoi(argv[1]);
  const int gripper_command = std::stoi(argv[2]);

  if (operating_mode == 1) {
    std::cout << "Operation Mode: Moving!\n";
  } else {
    std::cout << "Operation Mode: NOT moving ...\n";
  }

  initialize_robot_state();

  if (gripper_command == 1) {
    std::cout << "Opening/Closing gripper :)\n";
    close_gripper();
    std::cout << "Gripper Closed\n";
  }

  if (operating_mode == 1) {
    std::thread command_receiver_thread(receive_commands);
    std::thread state_publisher_thread(publish_state);
    std::thread robot_control_thread(move_end_effector);

    command_receiver_thread.join();
    state_publisher_thread.join();
    robot_control_thread.join();
  } else {
    std::thread command_receiver_thread(receive_commands);
    std::thread state_publisher_thread(publish_state);

    command_receiver_thread.join();
    state_publisher_thread.join();
  }

  std::cout << "Stopped getting incoming data\n";
  return EXIT_SUCCESS;
}
