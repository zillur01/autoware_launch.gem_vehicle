cmake_minimum_required(VERSION 3.14)
project(gem_ouster_pointcloud_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ouster_pointcloud_converter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "gem_ouster_pointcloud_converter::OusterPointCloudConverter"
  EXECUTABLE ouster_pointcloud_converter_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
