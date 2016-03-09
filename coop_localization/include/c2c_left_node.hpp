#ifndef C2C_LEFT_Node_HPP
#define C2C_LEFT_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

using std::string;


class C2C_LEFT_Node
{
public:
	C2C_LEFT_Node();
	ros::NodeHandle nh;

	// ROS message callback to ar_sys node
	void arsys_transform_callback(const geometry_msgs::TransformStamped& transformMsg);

	// functions
	tf::StampedTransform get_c2c(const tf::StampedTransform& stampedT_in);
	tf::StampedTransform merge_msgsToTF(const geometry_msgs::TransformStamped& msg1, const geometry_msgs::TransformStamped& msg2);
	tf::StampedTransform choose_c2c();
	tf::StampedTransform& stampedTF_filter(tf::StampedTransform &steady, const tf::StampedTransform &fresh);
	void read_robotModel_input();

private:
	ros::Subscriber transform_sub;			// subscribing 	to 	/ar_sys_.../transform
	ros::Publisher transform_pub;			// publishing 	to 	/c2c
	tf::TransformBroadcaster tf_broadcaster;	// broadcaster for rviz

	// initial transform for filter
	tf::StampedTransform steady_transform = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), "a_center", "c2c_left");

	// changeable parameters
	string side = "left";
	double filter_change_rate = 1;		// see C2C_LEFT_Node::stampedTF_filter
	double weight_of_center = 0.8;			// see C2C_LEFT_Node::merge_msgsToTF

	// fixed parameters
	int count = 8;				// board count
	int last_entry = -1;
	tf::StampedTransform c2c_buffer [2];
	geometry_msgs::TransformStamped collected_msgs [8];		// (0,1) --> front  (2,3) --> right  (4,5) --> back  (6,7) --> left
	bool entry_changed [8];
	tf::Transform apollon [7];		// storing robot model information
	tf::Transform boreas [7];

};

#endif // C2C_LEFT_Node_HPP
