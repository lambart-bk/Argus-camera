cmake_minimum_required(VERSION 2.8.3)
project(camera_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/camera_node/cmake" "${CMAKE_MODULE_PATH}")
MESSAGE(${CMAKE_SOURCE_DIR})
find_package(Argus REQUIRED)
find_package(Argus REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_test
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)



include_directories(
    ${catkin_INCLUDE_DIRS}
    "/usr/local/include"
    "/usr/include"
    $(ARGUS_UTILS_DIR)
    ${ARGUS_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/camera_node/include/
    ${CMAKE_SOURCE_DIR}/camera_node/utils
    ${OpenCV_INCLUDE_DIRS}
    )
link_directories("/usr/lib/aarch64-linux-gnu/tegra")

set(SOURCES
    src/main.cpp
    src/CameraAPI.cpp
    src/Dispathcher.cpp
    )

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
     ${catkin_LIBRARIES}
    -lnvbuf_utils
    ${ARGUS_LIBRARIES}
    -lnveglstreamproducer 
    -lpthread 
    -lv4l2 -lEGL -lGLESv2 -lX11 
    -lnvjpeg -lnveglstream_camconsumer
    ${OpenCV_LIBRARIES}
    )


