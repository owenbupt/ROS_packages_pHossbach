#ifndef COOP_FUSION_Node_HPP
#define COOP_FUSION_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

using std::string;


class COOP_FUSION_Node
{
public:
	COOP_FUSION_Node();
	ros::NodeHandle nh;

	void c2c_callback(const geometry_msgs::TransformStamped& c2c_transformMsg);	// ROS callback
	void libViso2_callback(const geometry_msgs::PoseStamped& libViso2_poseMsg);	// ROS callback
	tf::StampedTransform stampedTF_fusion(tf::StampedTransform transform1, tf::StampedTransform transform2);

private:
	ros::Subscriber c2c_sub;
	ros::Subscriber libViso2_sub;
	ros::Publisher odom_reset_pub;
	ros::Publisher fusion_pub;
	tf::TransformBroadcaster tf_broadcaster;	// for vizualisation in rviz

	// Initial stampedTransform for filter
	//tf::StampedTransform steady_transform = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), "a_center", "c2c");
	tf::StampedTransform memory[2];	// memory for the incoming stampedTransforms
	tf::Quaternion q0;

	double init_A[2] = {2.4, 1};
	double init_B[2] = {2.4, 2.4};
	double odom_correction[7] = {0,0,0,0,0,0,0};
	bool odom_corrected = false;

	double weight_of_c2c = 0.8;		// possibility for weighting c2c method (default = 0.8)

};

#endif // COOP_FUSION_Node_HPP
