cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0057)
cmake_policy(SET CMP0057 NEW)
endif()
project(vins_estimator)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV 4 REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
)

#catkin_package()

add_library(vins_estimator
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    )


target_link_libraries(vins_estimator ${OpenCV_LIBS} ${CERES_LIBRARIES} rclcpp::rclcpp) 

ament_target_dependencies(vins_estimator rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_model image_transport)

#add_executable(vins_node src/rosNodeTest.cpp)
#ament_target_dependencies(vins_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_model image_transport)
#target_link_libraries(vins_node vins_estimator) 

#add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
#ament_target_dependencies(kitti_odom_test rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_model image_transport)
#target_link_libraries(kitti_odom_test vins_estimator) 

#add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
#ament_target_dependencies(kitti_gps_test rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_model image_transport)
#target_link_libraries(kitti_gps_test vins_estimator) 

# Install
# install(TARGETS
#     vins_node
#     kitti_odom_test
#     kitti_gps_test
#     DESTINATION lib/${PROJECT_NAME}
# )

# Install binaries
install(TARGETS vins_estimator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# install(TARGETS  vins_node kitti_odom_test kitti_gps_test
#     RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY
    launch
    # config
    # rviz
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()