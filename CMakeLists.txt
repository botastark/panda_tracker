cmake_minimum_required(VERSION 3.16)
project(panda_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PANDA_TRACKER_BUILD_ROBOT "Build libfranka programs" ON)

find_package(Threads REQUIRED)

# ------------------------------------------------------------------
# Shared C++ code
# ------------------------------------------------------------------

add_library(panda_tracker_core
    cpp/src/task_pose_protocol.cpp
    cpp/src/tracker_receiver.cpp
    cpp/src/pbvs.cpp
    cpp/src/pbvs_config.cpp
)

target_include_directories(panda_tracker_core
    PUBLIC cpp/include
)

target_link_libraries(panda_tracker_core
    PUBLIC Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(panda_tracker_core PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

# ------------------------------------------------------------------
# Tracker test sender
# ------------------------------------------------------------------

add_executable(fake_task_pose_sender
    cpp/src/fake_task_pose_sender.cpp
)

target_link_libraries(fake_task_pose_sender
    PRIVATE panda_tracker_core
)

# ------------------------------------------------------------------
# Panda / libfranka programs
# ------------------------------------------------------------------

if(PANDA_TRACKER_BUILD_ROBOT)

    find_package(Franka REQUIRED)

    add_executable(pbvs_observer
        cpp/src/pbvs_observer.cpp
    )

    target_link_libraries(pbvs_observer
        PRIVATE
            panda_tracker_core
            Franka::Franka
    )

    add_executable(franka_frame_dump
        cpp/src/franka_frame_dump.cpp
    )

    target_link_libraries(franka_frame_dump
        PRIVATE
            panda_tracker_core
            Franka::Franka
    )

    add_executable(robot_pose_streamer
        cpp/src/robot_pose_streamer.cpp
    )

    target_link_libraries(robot_pose_streamer
        PRIVATE
            Franka::Franka
            Threads::Threads
    )

    add_executable(pbvs_robot
        cpp/src/pbvs_robot.cpp
    )

    target_link_libraries(pbvs_robot
        PRIVATE
            panda_tracker_core
            Franka::Franka
            Threads::Threads
    )

    add_executable(touch_calibration_collect
        cpp/src/touch_calibration_collect.cpp
    )

    target_link_libraries(touch_calibration_collect
        PRIVATE
            panda_tracker_core
            Franka::Franka
    )
    add_executable(pbvs_robot_flange
      cpp/src/pbvs_robot_flange.cpp
    )

    target_link_libraries(pbvs_robot_flange
      PRIVATE
        panda_tracker_core
        Franka::Franka
        Threads::Threads
    )

    target_compile_features(pbvs_robot_flange PRIVATE cxx_std_17)

    add_executable(pbvs_observer_full_pose
        cpp/src/pbvs_observer_full_pose.cpp
    )
    target_link_libraries(pbvs_observer_full_pose
        PRIVATE
            panda_tracker_core
            Franka::Franka
            Threads::Threads
    )
    add_executable(pbvs_observer_axis_diagnostics
        cpp/src/pbvs_observer_axis_diagnostics.cpp
    )
    target_link_libraries(pbvs_observer_axis_diagnostics
        PRIVATE
            panda_tracker_core
            Franka::Franka
            Threads::Threads
    )

endif()