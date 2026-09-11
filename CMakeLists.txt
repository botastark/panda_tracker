cmake_minimum_required(VERSION 3.16)

project(panda_position_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Franka REQUIRED)

add_executable(pbvs_robot_position
    cpp/src/geometry.cpp
    cpp/src/task_pose_protocol.cpp
    cpp/src/tracker_receiver.cpp
    cpp/src/position_tracking_config.cpp
    cpp/src/tracker_position_filter.cpp
    cpp/src/position_servo.cpp
    cpp/src/robot_safety.cpp
    cpp/src/pbvs_robot_position.cpp
)

target_include_directories(pbvs_robot_position
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
)

target_link_libraries(pbvs_robot_position
    PRIVATE
        Franka::Franka
        Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pbvs_robot_position PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()
