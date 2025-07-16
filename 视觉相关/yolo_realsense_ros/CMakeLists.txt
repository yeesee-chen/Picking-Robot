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
    # 如果是点位方案换成点位的py文件
    # scripts/yolo_node_point.py
    scripts/yolo_node_fix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)