# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

cmake_minimum_required(VERSION 3.8)
project(task_1b)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  task_1b/inverse_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inverse_kinematics
)

install(PROGRAMS
  bin/cmd_vel_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
