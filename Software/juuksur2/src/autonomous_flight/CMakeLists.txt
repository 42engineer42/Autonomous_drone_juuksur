cmake_minimum_required(VERSION 2.8.3)
project(autonomous_flight)

add_compile_options(-gdwarf-4)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++11>)

find_package(catkin REQUIRED
    roscpp
    roslib
    dev_msgs
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

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l)
    add_compile_definitions(HARDWARE_RASPI_3=1)
else()
    add_compile_definitions(HARDWARE_SIMULATOR=1)
endif()

add_executable(${PROJECT_NAME}_node src/autonomous_flight_node.cpp src/autonomous_controller.cpp src/localization.cpp src/BezierSpline.cpp src/params.cpp src/feature_tracker.cpp)
target_link_libraries(autonomous_flight_node ${OpenCV_LIBS} ${catkin_LIBRARIES} ttmath)
