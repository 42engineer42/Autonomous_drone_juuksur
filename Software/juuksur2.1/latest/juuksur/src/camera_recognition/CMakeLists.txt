cmake_minimum_required(VERSION 2.8.3)
project(camera_recognition)

find_package(
  catkin REQUIRED
  roscpp
  roslib
  std_msgs
  sensor_msgs
  cv_bridge
  dev_msgs
  image_transport
  ttmath
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include/
)

add_executable(camera_recognition_node src/camera_recognition_node.cpp)
target_link_libraries(camera_recognition_node ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp ttmath)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

