cmake_minimum_required(VERSION 3.0.2)
project(theta_s_ros)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  sensor_msgs
  cv_bridge
  pluginlib
  nodelet
)
find_package(Boost REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES theta_s_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/image_converter_nodelet.cpp
  include/opencv_theta_s/EquirectangularConversion/ThetaConversion.cpp
  include/panorama2cubemap/src/Panorama2Cubemap.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(
  image_converter
  src/image_converter.cpp
  include/opencv_theta_s/EquirectangularConversion/ThetaConversion.cpp
  include/panorama2cubemap/src/Panorama2Cubemap.cpp
)
target_link_libraries(image_converter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(
  rosbag_converter
  src/rosbag_converter.cpp
  include/opencv_theta_s/EquirectangularConversion/ThetaConversion.cpp
  include/panorama2cubemap/src/Panorama2Cubemap.cpp
)
target_link_libraries(rosbag_converter
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
