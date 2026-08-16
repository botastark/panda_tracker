cmake_minimum_required(VERSION 3.16)

project(panda_tracker VERSION 0.1.0 LANGUAGES CXX)

option(
  PANDA_TRACKER_BUILD_ROBOT
  "Build the read-only libfranka hardware observer"
  ON
)

include(CTest)
include(GNUInstallDirs)

find_package(Threads REQUIRED)

add_library(panda_tracker_protocol STATIC
  cpp/src/task_pose_protocol.cpp
)

target_include_directories(panda_tracker_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(panda_tracker_protocol PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(panda_tracker_protocol PRIVATE
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

if(BUILD_TESTING)
  add_executable(test_task_pose_protocol
    cpp/tests/test_task_pose_protocol.cpp
  )
  target_link_libraries(test_task_pose_protocol
    PRIVATE panda_tracker_protocol
  )
  add_test(NAME task_pose_protocol COMMAND test_task_pose_protocol)
endif()

add_executable(fake_task_pose_sender
  cpp/src/fake_task_pose_sender.cpp
)
target_link_libraries(fake_task_pose_sender
  PRIVATE
    Threads::Threads
    panda_tracker_protocol
)
target_compile_features(fake_task_pose_sender PRIVATE cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(fake_task_pose_sender PRIVATE
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

install(TARGETS fake_task_pose_sender
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(PANDA_TRACKER_BUILD_ROBOT)
  if(NOT TARGET Franka::Franka)
    find_package(Franka REQUIRED)
  endif()

  add_executable(pbvs_observer
    cpp/src/pbvs_observer.cpp
  )
  target_link_libraries(pbvs_observer
    PRIVATE
      Franka::Franka
      Threads::Threads
      panda_tracker_protocol
  )
  target_compile_features(pbvs_observer PRIVATE cxx_std_17)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pbvs_observer PRIVATE
      -Wall
      -Wextra
      -Wpedantic
    )
  endif()

  install(TARGETS pbvs_observer
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()