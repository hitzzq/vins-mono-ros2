#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <visualization_msgs/msg/marker.hpp>
//#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>
#if 1
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry;
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_relo_path;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_margin_cloud;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_relo_relative_pose;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;


extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic;
#endif
extern nav_msgs::msg::Path path;
extern int IMAGE_ROW, IMAGE_COL;
#if 0
extern rclcpp::Publisher<>::SharedPtr pub_odometry;
extern rclcpp::Publisher<>::SharedPtr pub_path, pub_pose;
extern rclcpp::Publisher<>::SharedPtr pub_cloud, pub_map;
extern rclcpp::Publisher<>::SharedPtr pub_key_poses;
extern rclcpp::Publisher<>::SharedPtr pub_ref_pose, pub_cur_pose;
extern rclcpp::Publisher<>::SharedPtr pub_key;
extern nav_msgs::msg::Path path;
extern rclcpp::Publisher<>::SharedPtr pub_pose_graph;
extern int IMAGE_ROW, IMAGE_COL;
#endif
void registerPub(rclcpp::Node::SharedPtr n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);