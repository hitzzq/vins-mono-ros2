cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0057)
cmake_policy(SET CMP0057 NEW)
endif()

project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    std_msgs
#    sensor_msgs
#    cv_bridge
#    camera_model
#    )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(camera_model REQUIRED)
find_package(OpenCV 4 REQUIRED)

#catkin_package()

include_directories("../camera_model/include")
#include_directories(
#    ${catkin_INCLUDE_DIRS}
#    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
#  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
message(STATUS "OpenCV:${OpenCV_LIBS}")
add_library(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )
ament_target_dependencies(feature_tracker std_msgs cv_bridge camera_model)
#target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(feature_tracker ${OpenCV_LIBS} rclcpp::rclcpp)
install(TARGETS feature_tracker
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_package()