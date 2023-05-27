#pragma once

#include <rclcpp/rclcpp.hpp>
#if 0
#include <std_msgs/msg/color_rgba.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/msg/marker_array.hpp>
#endif
//#include <std_msgs/ColorRGBA.h>
#include <std_msgs/msg/color_rgba.hpp>
//#include <visualization_msgs/Marker.h>
#include <visualization_msgs/msg/marker.hpp>
//#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "../parameters.h"

class CameraPoseVisualization {
public:
	std::string m_marker_ns;

	CameraPoseVisualization(float r, float g, float b, float a);
	
	void setImageBoundaryColor(float r, float g, float b, float a=1.0);
	void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
	void setScale(double s);
	void setLineWidth(double width);

	void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
	void reset();

	void publish_by(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, const std_msgs::msg::Header& header);
	void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
	void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
	//void add_image(const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const cv::Mat &src);
	void publish_image_by( rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const std_msgs::msg::Header &header);
private:
	std::vector<visualization_msgs::msg::Marker> m_markers;
	std_msgs::msg::ColorRGBA m_image_boundary_color;
	std_msgs::msg::ColorRGBA m_optical_center_connector_color;
	double m_scale;
	double m_line_width;
	visualization_msgs::msg::Marker image;
	int LOOP_EDGE_NUM;
	int tmp_loop_edge_num;

	static const Eigen::Vector3d imlt;
	static const Eigen::Vector3d imlb;
	static const Eigen::Vector3d imrt;
	static const Eigen::Vector3d imrb;
	static const Eigen::Vector3d oc  ;
	static const Eigen::Vector3d lt0 ;
	static const Eigen::Vector3d lt1 ;
	static const Eigen::Vector3d lt2 ;
};
