cmake_minimum_required(VERSION 2.8.3)
project(simulator_camera)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    image_transport
    cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(simulator_camera_node src/camera_node.cpp)
target_link_libraries(simulator_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

