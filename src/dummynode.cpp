#include <ros/ros.h>
#include <ros/time.h>
#include <frame_msgs/TrackedPersons.h>
#include <frame_msgs/TrackedPerson.h>
#include <string>

int main(int argc, char** argv)
{
	std::string tf_id_global("tf_qolo_world");

	ros::init(argc, argv, "dummy_tracker");
	ros::NodeHandle node;

	ros::Publisher tracksPub(node.advertise<frame_msgs::TrackedPersons>(
		"rwth_tracker/tracked_persons", 1));

	unsigned int tracks_msg_counter(0);
	frame_msgs::TrackedPersons tracks_msg;
	tracks_msg.header.frame_id = tf_id_global; //"tf_front_lidar";
	frame_msgs::TrackedPerson person_msg;
	person_msg.is_occluded = false;
	person_msg.detection_id = 0;

	ROS_INFO("Dummy tracker started.");

	ros::spinOnce();

	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		tracks_msg.tracks.clear();
		{
			person_msg.track_id = 0;
			person_msg.is_matched = true;
			person_msg.age = ros::Duration(0.05*tracks_msg_counter);

			person_msg.pose.pose.position.x = 3.f;
			person_msg.pose.pose.position.y = 0.f;
			person_msg.pose.pose.position.z = 0.f;
			person_msg.twist.twist.linear.x = 0.f;
			person_msg.twist.twist.linear.y = 0.f;
			person_msg.twist.twist.linear.z = 0.f;
			tracks_msg.tracks.push_back(person_msg);
		}

		tracks_msg.header.seq = tracks_msg_counter++;
		tracks_msg.header.stamp = ros::Time::now();

		tracksPub.publish(tracks_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}