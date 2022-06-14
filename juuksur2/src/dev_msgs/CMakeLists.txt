cmake_minimum_required(VERSION 2.8.3)
project(dev_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  APoint.msg
  AttitudeData.msg
  VisionData.msg
  PathData.msg
  MoveData.msg
  SwitchData.msg
  KeyData.msg
  CameraFrame.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
