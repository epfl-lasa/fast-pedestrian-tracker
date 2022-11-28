#include "MarkRviz.hpp"

MarkSegments::MarkSegments(ros::NodeHandle& ros_node_handle, const std::string& tf_id, const std::string& topic)
{
	m_segments_publisher = ros_node_handle.advertise<visualization_msgs::Marker>(
		topic, 10);

	// frame in which you have cluster coordinates
	m_line_list.header.frame_id = tf_id;
	//marker type , scale,colour
	m_line_list.type = visualization_msgs::Marker::LINE_LIST;
	m_line_list.scale.x = 0.1;
	m_line_list.color.r = 1.0;
	m_line_list.color.a = 1.0;
}

void MarkSegments::visualize(const std::vector<std::vector<Point2D> >& point_segments, const ros::Time& stamp)
{
	m_line_list.header.stamp = stamp;
	m_line_list.points.resize(0);
	geometry_msgs::Point line_start, line_end;
	line_start.z = 0.0;
	line_end.z = 0.0;
	for (int i = 0; i < point_segments.size(); i++)
	{
		line_start.x = point_segments[i][0].x;
		line_start.y = point_segments[i][0].y;
		line_end.x = point_segments[i].back().x;
		line_end.y = point_segments[i].back().y;	
		m_line_list.points.push_back(line_start);
		m_line_list.points.push_back(line_end);
	}
	// publish marker
	m_segments_publisher.publish(m_line_list);
}

MarkCircles::MarkCircles(ros::NodeHandle& ros_node_handle, float rMax, float costMax,  const std::string& tf_id, const std::string& topic)
: m_r_max(rMax)
, costMax(costMax)
{
	m_circles_publisher = ros_node_handle.advertise<visualization_msgs::MarkerArray>(
		topic, 10);

	m_circle_template.header.frame_id = tf_id;
	m_circle_template.type = visualization_msgs::Marker::CYLINDER;
	m_circle_template.color.g = 1.0;
	m_circle_template.color.a = 1.0;
	m_circle_template.scale.z = 0.05;
	m_circle_template.pose.position.z = 0.0;
}

void MarkCircles::visualize(const std::vector<Circle>& circles, const ros::Time& stamp)
{
	m_circle_template.header.stamp = stamp;
	if (circles.size() > m_circle_list.markers.size()) // to hide old circles (see below)
		m_circle_list.markers.resize(circles.size());
	for (int i = 0; i < circles.size(); i++)
	{
		m_circle_template.id = i;
		if (circles[i].r < m_r_max && circles[i].cost > costMax)
			m_circle_template.scale.x = m_circle_template.scale.y = circles[i].r*2.0;
		else
			m_circle_template.scale.x = m_circle_template.scale.y = 0.0;
		m_circle_template.pose.position.x = circles[i].c.x;
		m_circle_template.pose.position.y = circles[i].c.y;
		m_circle_list.markers[i] = m_circle_template;
	}

	// this is just to hide old circles
	for (int i = circles.size(); i < m_circle_list.markers.size(); i++)
	{
		m_circle_template.id = i;
		m_circle_template.scale.x = m_circle_template.scale.y = 0.0;
		m_circle_list.markers[i] = m_circle_template;
	}

	/*m_circle_list.markers.resize(2);
	m_circle_list.markers[0].pose.position.x = 1.0;
	m_circle_list.markers[0].pose.position.y = 0.0;
	m_circle_list.markers[0].pose.position.z = 0.0;
	m_circle_list.markers[0].scale.x = 0.5;
	m_circle_list.markers[0].scale.y = 0.5;
	m_circle_list.markers[0].scale.z = 0.5;
	m_circle_list.markers[0].header.frame_id = "sick_laser_front";
	m_circle_list.markers[0].type = visualization_msgs::Marker::CYLINDER;
	m_circle_list.markers[0].color.g = 1.0;
	m_circle_list.markers[0].color.a = 1.0;
	m_circle_list.markers[1].pose.position.x = -1.0;
	m_circle_list.markers[1].pose.position.y = 0.0;
	m_circle_list.markers[1].pose.position.z = 0.0;
	m_circle_list.markers[1].scale.x = 0.5;
	m_circle_list.markers[1].scale.y = 0.5;
	m_circle_list.markers[1].scale.z = 0.5;
	m_circle_list.markers[1].header.frame_id = "sick_laser_front";
	m_circle_list.markers[1].type = visualization_msgs::Marker::CYLINDER;
	m_circle_list.markers[1].color.g = 1.0;
	m_circle_list.markers[1].color.a = 1.0;*/

	// publish marker array
	m_circles_publisher.publish(m_circle_list);
}
