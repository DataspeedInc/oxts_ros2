cmake_minimum_required(VERSION 3.20)
project(oxts_ros2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/oxts_driver.cpp
  src/NComRxC.c
  src/kinematics_transformer.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::OxtsDriver"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)