cmake_minimum_required(VERSION 3.0.2)
project(calibration_test_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

