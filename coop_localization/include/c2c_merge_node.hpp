#ifndef C2C_MERGE_Node_HPP
#define C2C_MERGE_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

using std::string;


class C2C_MERGE_Node
{
public:
	C2C_MERGE_Node();
	ros::NodeHandle nh;

	void c2c_callback(const geometry_msgs::TransformStamped& c2c_transformMsg);	// ROS callback
	tf::StampedTransform& stampedTF_filter(tf::StampedTransform &steady, const tf::StampedTransform &fresh);
	tf::StampedTransform stampedTF_merge(tf::StampedTransform transform1, tf::StampedTransform transform2);

private:
	ros::Subscriber c2c_sub;
	ros::Publisher c2c_merged_pub;
	tf::TransformBroadcaster tf_broadcaster;	// for vizualisation in rviz

	// Initial stampedTransform for filter
	tf::StampedTransform steady_transform = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(0.0), "a_center", "c2c");
	tf::StampedTransform c2c_in [2];	// memory for incoming stampedTransforms

	double weight_of_right = 0.5;		// possibility for weighting one side (default = 0.5)
	double filter_change_rate = 0.2;
	bool reseted1 = false;
	bool reseted2 = false;

};

#endif // C2C_MERGE_Node_HPP
