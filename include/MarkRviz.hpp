#ifndef MARK_RVIZ_HPP
#define MARK_RVIZ_HPP

#include "common.hpp"

#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <string>

class MarkSegments
{
public:
	MarkSegments(ros::NodeHandle& ros_node_handle, const std::string& tf_id, const std::string& topic);

	void visualize(const std::vector<std::vector<Point2D> >& point_segments, const ros::Time& stamp);

private:
	ros::Publisher m_segments_publisher;
	visualization_msgs::Marker m_line_list;
};

class MarkCircles
{
public:
	MarkCircles(ros::NodeHandle& ros_node_handle, float rMax, float costMax, const std::string& tf_id, const std::string& topic);

	void visualize(const std::vector<Circle>& circles, const ros::Time& stamp);

	const float m_r_max, costMax;

private:
	ros::Publisher m_circles_publisher;
	visualization_msgs::MarkerArray m_circle_list;
	visualization_msgs::Marker m_circle_template;
};

#endif