cmake_minimum_required(VERSION 3.0.2)
project(raspi_telemetry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++11>)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 roslib
 std_msgs
 sensor_msgs
 cv_bridge
 image_transport
)
find_package(OpenCV 3 REQUIRED)

catkin_package(
)

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
/opt/vc/include
/opt/vc/src/hello_pi/libs/ilclient
)

add_compile_options(-DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM)

add_executable(${PROJECT_NAME}_node src/raspi_telemetry_node.cpp)

#set(CMAKE_VERBOSE_MAKEFILE on)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l)
    add_compile_definitions(HARDWARE_RASPI_3=1)
else()
    add_compile_definitions(HARDWARE_SIMULATOR=1)
endif()

message( "Processor " "${CMAKE_SYSTEM_PROCESSOR}" )
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l)
    target_link_directories(${PROJECT_NAME}_node PRIVATE /opt/vc/lib)
    target_sources(${PROJECT_NAME}_node PRIVATE src/encoder.c)
    target_sources(${PROJECT_NAME}_node PRIVATE /opt/vc/src/hello_pi/libs/ilclient/ilclient.c)
    target_link_libraries(${PROJECT_NAME}_node openmaxil bcm_host vcos vchiq_arm )

    message("Using Videocore IV video encoder")
else()
    target_sources(${PROJECT_NAME}_node PRIVATE src/mock_encoder.c)
endif()

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
${OpenCV_LIBS}
)
