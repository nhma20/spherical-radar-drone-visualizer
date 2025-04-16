cmake_minimum_required(VERSION 3.5)
project(spherical-radar-drone-visualizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(look_ahead_cone_republisher
  src/look_ahead_cone_republisher.cpp
)
  ament_target_dependencies(look_ahead_cone_republisher
  rclcpp
  ament_index_cpp
  visualization_msgs
)
install(TARGETS look_ahead_cone_republisher DESTINATION lib/${PROJECT_NAME})


# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime)# eigen3_cmake_module Eigen3 geometry_msgs sensor_msgs)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime)# Eigen3 geometry_msgs sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install the mesh directory
install(DIRECTORY
  mesh
  DESTINATION share/${PROJECT_NAME}/
)

# install the urdf directory
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# install the rviz directory
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


