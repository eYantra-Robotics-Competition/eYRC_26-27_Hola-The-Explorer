# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

cmake_minimum_required(VERSION 3.8)
project(task_1c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  task_1c/sim_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_control
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/shape_service.cpp")
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(shape_interface REQUIRED)

  add_executable(shape_service src/shape_service.cpp)
  ament_target_dependencies(shape_service rclcpp std_msgs shape_interface)
  install(TARGETS shape_service DESTINATION lib/${PROJECT_NAME})
else()
  install(PROGRAMS bin/shape_service DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
