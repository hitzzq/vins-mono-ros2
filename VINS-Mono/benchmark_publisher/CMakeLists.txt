cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0057)
cmake_policy(SET CMP0057 NEW)
endif()
project(benchmark_publisher)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS "-std=c++17 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -rdynamic")


#find_package(catkin REQUIRED COMPONENTS
 #   roscpp
 #   tf
 #   )
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#catkin_package()
#include_directories(${catkin_INCLUDE_DIRS}) 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
#  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(benchmark_publisher
    src/benchmark_publisher_node.cpp
    )
ament_target_dependencies(benchmark_publisher rclcpp rcpputils nav_msgs tf2 tf2_ros)
#target_link_libraries(benchmark_publisher ${catkin_LIBRARIES})
#target_link_libraries(benchmark_publisher rclcpp::rclcpp)

install(TARGETS
    benchmark_publisher
    DESTINATION lib/${PROJECT_NAME}
    )


install(TARGETS benchmark_publisher
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS benchmark_publisher
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY cmake/
    DESTINATION cmake/
)
# Install launch files
install(DIRECTORY
    launch
    # config
    # rviz
    DESTINATION share/${PROJECT_NAME}
    )
ament_package()