#include "panda_tracker/pbvs.h"

#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using panda_tracker::Transform;

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string output_path{};
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto next_value = [&](const char* option) -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument(std::string("Missing value for ") + option);
      }
      return argv[index];
    };
    if (argument == "--robot-ip") {
      options.robot_ip = next_value("--robot-ip");
    } else if (argument == "--output") {
      options.output_path = next_value("--output");
    } else if (argument == "--help" || argument == "-h") {
      std::cout
          << "Read one Panda state and audit libfranka frame transforms.\n\n"
          << "This program never starts a control loop and never changes the "
             "robot configuration.\n\n"
          << "Usage: " << argv[0] << " [options]\n"
          << "  --robot-ip IP   FCI address (default 172.16.0.2)\n"
          << "  --output PATH   Also write a JSON snapshot\n"
          << "  --help, -h      Show this message\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown option: " + argument);
    }
  }
  return options;
}

Transform from_franka(const std::array<double, 16>& value) {
  return panda_tracker::franka_column_major_transform(value);
}

double translation_norm(const Transform& transform) {
  return panda_tracker::vector_norm(
      panda_tracker::transform_translation(transform));
}

double rotation_angle(const Transform& transform) {
  return panda_tracker::vector_norm(
      panda_tracker::so3_log(panda_tracker::transform_rotation(transform)));
}

void print_transform(const char* name, const Transform& transform) {
  std::cout << name << " (row-major display):\n";
  for (std::size_t row = 0; row < 4; ++row) {
    std::cout << "  [";
    for (std::size_t column = 0; column < 4; ++column) {
      if (column != 0) {
        std::cout << ", ";
      }
      std::cout << std::setw(12) << transform[row * 4 + column];
    }
    std::cout << "]\n";
  }
}

void write_json_transform(
    std::ostream& output,
    const char* name,
    const Transform& transform,
    bool trailing_comma = true) {
  output << "  \"" << name << "\": [\n";
  for (std::size_t row = 0; row < 4; ++row) {
    output << "    [";
    for (std::size_t column = 0; column < 4; ++column) {
      if (column != 0) {
        output << ", ";
      }
      output << transform[row * 4 + column];
    }
    output << "]" << (row == 3 ? "\n" : ",\n");
  }
  output << "  ]" << (trailing_comma ? ",\n" : "\n");
}

void write_snapshot(
    const std::string& path,
    const Transform& O_T_EE,
    const Transform& F_T_EE,
    const Transform& F_T_NE,
    const Transform& NE_T_EE,
    const Transform& EE_T_K,
    const Transform& O_T_F,
    const Transform& F_T_H_sim,
    double closure_translation_m,
    double closure_angle_rad,
    double simulation_translation_m,
    double simulation_angle_rad) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Cannot open output file: " + path);
  }
  output << std::setprecision(17);
  output << "{\n"
         << "  \"schema\": \"panda_tracker_franka_frames_v1\",\n"
         << "  \"matrix_convention\": \"T_XY is frame Y expressed in X\",\n"
         << "  \"matrix_storage\": \"row-major JSON; libfranka input was "
            "column-major\",\n";
  write_json_transform(output, "O_T_EE", O_T_EE);
  write_json_transform(output, "F_T_EE", F_T_EE);
  write_json_transform(output, "F_T_NE", F_T_NE);
  write_json_transform(output, "NE_T_EE", NE_T_EE);
  write_json_transform(output, "EE_T_K", EE_T_K);
  write_json_transform(output, "O_T_F_derived", O_T_F);
  write_json_transform(output, "F_T_H_mujoco_reference", F_T_H_sim);
  output << "  \"F_T_EE_chain_translation_residual_m\": "
         << closure_translation_m << ",\n"
         << "  \"F_T_EE_chain_angle_residual_rad\": "
         << closure_angle_rad << ",\n"
         << "  \"F_T_EE_vs_mujoco_hand_translation_difference_m\": "
         << simulation_translation_m << ",\n"
         << "  \"F_T_EE_vs_mujoco_hand_angle_difference_rad\": "
         << simulation_angle_rad << "\n"
         << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  constexpr double kRadiansToDegrees = 57.2957795130823208768;
  try {
    const Options options = parse_options(argc, argv);

    std::cout
        << "READ-ONLY LIBFRANKA FRAME AUDIT\n"
        << "One Robot::readOnce() call; no controller and no configuration "
           "write.\n";

    franka::Robot robot(options.robot_ip);
    const franka::RobotState state = robot.readOnce();

    const Transform O_T_EE = from_franka(state.O_T_EE);
    const Transform F_T_EE = from_franka(state.F_T_EE);
    const Transform F_T_NE = from_franka(state.F_T_NE);
    const Transform NE_T_EE = from_franka(state.NE_T_EE);
    const Transform EE_T_K = from_franka(state.EE_T_K);
    const Transform O_T_F = panda_tracker::multiply_transform(
        O_T_EE, panda_tracker::invert_transform(F_T_EE));
    const Transform F_T_EE_from_chain = panda_tracker::multiply_transform(
        F_T_NE, NE_T_EE);
    const Transform chain_delta = panda_tracker::multiply_transform(
        panda_tracker::invert_transform(F_T_EE), F_T_EE_from_chain);

    // MuJoCo Menagerie franka_emika_panda/panda.xml body "hand" relative
    // to the flange: pos="0 0 0.107", quat="0.9238795 0 0 -0.3826834".
    const Transform F_T_H_sim{{
        0.7071067811865476, 0.7071067811865475, 0.0, 0.0,
        -0.7071067811865475, 0.7071067811865476, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.107,
        0.0, 0.0, 0.0, 1.0,
    }};
    const Transform simulation_delta = panda_tracker::multiply_transform(
        panda_tracker::invert_transform(F_T_EE), F_T_H_sim);

    std::cout << std::fixed << std::setprecision(9);
    print_transform("O_T_EE", O_T_EE);
    print_transform("F_T_EE", F_T_EE);
    print_transform("F_T_NE", F_T_NE);
    print_transform("NE_T_EE", NE_T_EE);
    print_transform("EE_T_K", EE_T_K);
    print_transform("O_T_F (derived)", O_T_F);
    print_transform("F_T_H_sim (MuJoCo reference)", F_T_H_sim);

    const double closure_translation_m = translation_norm(chain_delta);
    const double closure_angle_rad = rotation_angle(chain_delta);
    const double simulation_translation_m = translation_norm(simulation_delta);
    const double simulation_angle_rad = rotation_angle(simulation_delta);

    std::cout
        << "F_T_NE * NE_T_EE vs F_T_EE: translation residual="
        << closure_translation_m * 1000.0 << " mm, rotation residual="
        << closure_angle_rad * kRadiansToDegrees << " deg\n"
        << "F_T_EE vs MuJoCo hand: translation difference="
        << simulation_translation_m * 1000.0 << " mm, rotation difference="
        << simulation_angle_rad * kRadiansToDegrees << " deg\n";

    if (!options.output_path.empty()) {
      write_snapshot(
          options.output_path,
          O_T_EE,
          F_T_EE,
          F_T_NE,
          NE_T_EE,
          EE_T_K,
          O_T_F,
          F_T_H_sim,
          closure_translation_m,
          closure_angle_rad,
          simulation_translation_m,
          simulation_angle_rad);
      std::cout << "Wrote " << options.output_path << '\n';
    }

    if (closure_translation_m > 1e-9 || closure_angle_rad > 1e-9) {
      std::cerr << "FAIL: libfranka EE transform chain does not close.\n";
      return 2;
    }
    std::cout << "PASS: libfranka EE transform chain closes.\n";
    return 0;
  } catch (const franka::Exception& exception) {
    std::cerr << "libfranka error: " << exception.what() << '\n';
  } catch (const std::exception& exception) {
    std::cerr << "Error: " << exception.what() << '\n';
  }
  return 1;
}
