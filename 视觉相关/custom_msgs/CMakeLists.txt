cmake_minimum_required(VERSION 3.0.2)
project(yolo_realsense_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  message_filters
)

add_message_files(
  FILES
  detection3d.msg
  FruitStatusArray.msg
  MatureFruit.msg
  MatureFruitArray.msg
  MaturePoint.msg
  MaturePoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_runtime
  message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Use python3 explicitly
set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(CATKIN_PYTHON_VERSION 3)

catkin_install_python(
  PROGRAMS
    scripts/yolo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)