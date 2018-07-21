cmake_minimum_required(VERSION 2.8.3)
project(panda_teleop)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  rospy
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  panda_teleop_trajectory_keyboard
  include/panda_teleop/panda_teleop_trajectory_keyboard.h
  src/panda_teleop_trajectory_keyboard.cpp
)

target_link_libraries(
  panda_teleop_trajectory_keyboard
  ${catkin_LIBRARIES}
)
