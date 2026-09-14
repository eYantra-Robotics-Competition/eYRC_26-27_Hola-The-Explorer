# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

cmake_minimum_required(VERSION 3.8)
project(hb_description)

find_package(ament_cmake REQUIRED)
install(PROGRAMS
  bin/task1a_bridge
  bin/task1a_camera
  bin/task1c_bridge
  bin/task1c_camera
  bin/camera_stream
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY lib/
  DESTINATION lib
  USE_SOURCE_PERMISSIONS)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
