#include "geometry.hpp"
#include "tracker.hpp"
#include "segmentation.hpp"
#include "fit_circles.hpp"
#include "detector.hpp"

#include "MarkRviz.hpp"

#include "configuration.hpp"

#include <frame_msgs/TrackedPersons.h>
#include <frame_msgs/TrackedPerson.h>
#include <frame_msgs/DetectedPersons.h>
#include <frame_msgs/DetectedPerson.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/package.h>

//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

//#include <iostream>
//#include <fstream>
#include <vector>
#include <string>

#define _USE_MATH_DEFINES
#include <cmath>

Detector* detectorPtr;

tf2_ros::Buffer* tf_buffer_ptr;

struct DetectionFrame
{
	DetectionFrame(const std::string& tf_id, const ros::Time& stamp) 
	: tf_id(tf_id)
	, stamp(stamp) { }

	std::vector<Vector2d> detections;
	std::string tf_id;
	ros::Time stamp;
};

std::vector<DetectionFrame> detectionStack;

void importDetections(const frame_msgs::DetectedPersons::ConstPtr& msg)
{
	detectionStack.push_back(DetectionFrame(msg->header.frame_id,
		msg->header.stamp));
	for (const auto& d : msg->detections)
	{
		detectionStack.back().detections.push_back(Vector2d(
			d.pose.pose.position.x, d.pose.pose.position.y, true));
	}
	//ROS_INFO("Import at time %f", msg->header.stamp.toSec());
}

void detect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	detectionStack.push_back(DetectionFrame(msg->header.frame_id,
		msg->header.stamp));

	detectorPtr->detectLegsInRangeScan(msg->ranges,
		&detectionStack.back().detections);
	//ROS_INFO("Detect at time %f", msg->header.stamp.toSec());
}

int transformDetectionFrame(const DetectionFrame& detFrame,
	const std::string& tf_id_global, std::vector<Vector2d>* detectionsGlobal)
{
	geometry_msgs::TransformStamped transformStamped;
	try
	{
		transformStamped = tf_buffer_ptr->lookupTransform(tf_id_global,
			detFrame.tf_id, detFrame.stamp);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s excpetion, when looking up tf from %s to %s", 
			ex.what(), tf_id_global.c_str(), detFrame.tf_id.c_str());
		return 1;
	}
	tf2::Quaternion rotation(transformStamped.transform.rotation.x,
		transformStamped.transform.rotation.y,
		transformStamped.transform.rotation.z,
		transformStamped.transform.rotation.w);

	tf2::Vector3 translation(transformStamped.transform.translation.x,
		transformStamped.transform.translation.y,
		transformStamped.transform.translation.z);

	tf2::Transform tf(rotation, translation);

	for (const auto& det : detFrame.detections)
	{
		tf2::Vector3 p(tf*tf2::Vector3(det.x, det.y, 0.f));
		detectionsGlobal->push_back(Vector2d(p.getX(), p.getY(), det.userFlag));
	}
}

int main(int argc, char** argv)
{
	std::string tf_id_global("tf_qolo_world");

	ros::init(argc, argv, "fast_pedestrian_tracker");
	ros::NodeHandle node;

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
	tf_buffer_ptr = &tf_buffer;

	std::string pkgRoot(ros::package::getPath("fast-pedestrian-tracker"));
	Configuration config(pkgRoot.append("/config.txt"));
	if (!config.good)
	{
		ROS_WARN("Failed to read configuration file. Aborting.");
		return 1;
	}
	else
	{
		ROS_INFO("Parsed configuration file");
		config.print();
	}

	const unsigned int nConfirm(config.nConfirm);
	const unsigned int nConfirmSpecifically(config.nConfirmSpecifically);
	float dt = 0.05;
	Tracker tracker(dt, config.alpha, config.beta, config.gamma,
		config.rGate, config.lookBack, config.lookBackSpecific, (config.lookBackSpecific > 0));
	
	float abd_threshold = 10.0/360.0*2.0*3.141; // shallow angle threshold in [rad] 
	float range_noise = 0.03; // standard deviation in [m]
	float angular_step = 0.00700000021607; // 0.4/360.0*2.0*M_PI;
	
	AdaptiveBreakpointDetector abd(angular_step, abd_threshold, range_noise, -M_PI, 0.001, 200.0);
	
	FitCircles fc;

	float rMin(0.0);
	float rMax(config.rMax);
	float max_length(config.lMax);
	float min_length(config.lMin);
	float costMax(config.costMin);
	Detector detector(abd, fc, rMin, rMax, max_length, min_length, 0.99, costMax);
	
	detectorPtr = &detector;

	MarkSegments markSegs1(node, "tf_front_lidar", "segments1");
	MarkCircles markCircs1(node, rMax, costMax, "tf_front_lidar", "circles1");
	MarkSegments markSegs2(node, "tf_rear_lidar", "segments2");
	MarkCircles markCircs2(node, rMax, costMax, "tf_rear_lidar", "circles2");
	
	ros::Subscriber lrfSub1(node.subscribe("/front_lidar/scan", 1, detect));
	ros::Subscriber lrfSub2(node.subscribe("/rear_lidar/scan", 1, detect));
	ros::Subscriber spaamSub1, spaamSub2;
	if (config.importDetections)
	{
		spaamSub1 = node.subscribe("/drow_detected_persons_front", 
			1, importDetections);
		spaamSub2 = node.subscribe("/drow_detected_persons_rear", 
			1, importDetections);
	}

	ros::Publisher tracksPub(node.advertise<frame_msgs::TrackedPersons>(
		"rwth_tracker/tracked_persons", 1));

	unsigned int tracks_msg_counter(0);
	frame_msgs::TrackedPersons tracks_msg;
	tracks_msg.header.frame_id = tf_id_global; //"tf_front_lidar";
	frame_msgs::TrackedPerson person_msg;
	person_msg.is_occluded = false;
	person_msg.detection_id = 0;

	ROS_INFO("Fast pedestrian tracker started.");

	ros::spinOnce();

	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		std::vector<Vector2d> dets;
		for (const auto& detFr : detectionStack)
		{
			transformDetectionFrame(detFr, tf_id_global, &dets);
		}
		std::string last_det_tf_id;
		ros::Time last_det_stamp;
		if (detectionStack.size() != 0)
		{
			last_det_tf_id = detectionStack.back().tf_id;
			last_det_stamp = detectionStack.back().stamp;
		}
		detectionStack.clear();
		//if (detectionStack.size() != 0)
		//{
		//	transformDetectionFrame(detectionStack.back(), tf_id_global, &dets);
		//	detectionStack.pop_back();
		//}

		tracker.cycle(dets);

		if (!last_det_tf_id.empty())
		{
			if (last_det_tf_id == "tf_front_lidar")
			{
				markSegs1.visualize(detector.segsReduced, last_det_stamp);
				//markSegs.visualize(detector.segsNotOccluded);
				markCircs1.visualize(detector.fc.getLastCircles(), last_det_stamp);
			}
			else
			{
				markSegs2.visualize(detector.segsReduced, last_det_stamp);
				//markSegs.visualize(detector.segsNotOccluded);
				markCircs2.visualize(detector.fc.getLastCircles(), last_det_stamp);
			}
		}
		tracks_msg.tracks.clear();
		for (const auto& t : tracker.getTracks())
		{
			if (t.nObserved < nConfirm || 
				(t.nObservedSpecifically < nConfirmSpecifically && config.importDetections))
			{
				continue;
			}
			person_msg.track_id = t.id;
			person_msg.is_matched = (t.frame == tracker.getFrame());
			person_msg.age = ros::Duration(
				dt*(tracker.getFrame() - t.firstFrame));

			person_msg.pose.pose.position.x = t.fusion.p.x;
			person_msg.pose.pose.position.y = t.fusion.p.y;
			person_msg.pose.pose.position.z = 0.f;
			person_msg.twist.twist.linear.x = t.fusion.v.x;
			person_msg.twist.twist.linear.y = t.fusion.v.y;
			person_msg.twist.twist.linear.z = 0.f;
			tracks_msg.tracks.push_back(person_msg);
		}

		tracks_msg.header.seq = tracks_msg_counter++;
		tracks_msg.header.stamp = ros::Time::now();

		tracksPub.publish(tracks_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}


    //rosbag::Bag bag;
    //bag.open("../../data/lrf_mot_test.bag"); 

  	//std::ofstream myfile;
  	//myfile.open ("fusions.txt");


    //for(rosbag::MessageInstance const m: rosbag::View(bag))
    //{
    //	sensor_msgs::LaserScan::ConstPtr msg(m.instantiate<sensor_msgs::LaserScan>());
    //	if (msg != nullptr)
    //	{
	//		std::vector<Vector2d> detections;
	//		detector.detectLegsInRangeScan(msg->ranges, &detections);

	//		tracker.cycle(detections);
	//		for (const auto& trk : tracker.getTracks())
	//		{
	//			myfile << tracker.getFrame() << ", ";
	//			myfile << trk.id << ", ";
	//			myfile << trk.fusion.p.x << ", ";
	//			myfile << trk.fusion.p.y << std::endl;
	//		}
    //	}
    //}

  	//myfile.close();

    //bag.close();

	//ros::spin();
}
