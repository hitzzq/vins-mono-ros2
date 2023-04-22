cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0057)
cmake_policy(SET CMP0057 NEW)
endif()
project(ar_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -DEIGEN_DONT_PARALLELIZE")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  std_msgs
#  image_transport
#  sensor_msgs
#  cv_bridge
#  message_filters
#  camera_model
#)
#find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(camera_model REQUIRED)
find_package(OpenCV REQUIRED)

#catkin_package(
#)


#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

# include camera_models
include_directories("../camera_model/include")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
 # ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ar_demo_node src/ar_demo_node.cpp)

# target_link_libraries(ar_demo_node
#   ${catkin_LIBRARIES} ${OpenCV_LIBS}
# )

ament_target_dependencies(ar_demo_node rclcpp)
 target_link_libraries(ar_demo_node
  ${OpenCV_LIBS}
#  rclcpp::rclcpp
 )

ament_package()