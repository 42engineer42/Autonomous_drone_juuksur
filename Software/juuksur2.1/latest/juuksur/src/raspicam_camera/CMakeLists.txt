cmake_minimum_required(VERSION 2.8.3)
project(raspicam_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    image_transport
    cv_bridge    
)

add_subdirectory(raspicam)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES raspicam_camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 raspicam/src
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/raspicam_camera_node.cpp)
target_link_libraries(raspicam_camera_node raspicam)
target_link_libraries(raspicam_camera_node raspicam_cv)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

