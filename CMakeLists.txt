# Copyright 2025 Robotsix
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(robotsix_px4_sim_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModelInfo.msg"
  "action/StartSimulation.action"
  "action/StopSimulation.action"
)

# Install Python scripts
install(
  PROGRAMS
  scripts/start_simulation.py
  scripts/stop_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
