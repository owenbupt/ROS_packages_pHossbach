#ifndef FOLLOW_ROTATION_Node_HPP
#define FOLLOW_ROTATION_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

using std::string;


class FOLLOW_ROTATION_Node
{
public:
	FOLLOW_ROTATION_Node();
	ros::NodeHandle nh;

	void c2c_callback(const geometry_msgs::TransformStamped& c2c_transformMsg);	// ROS callback

private:
	ros::Subscriber c2c_sub;
	ros::Publisher follow_rotation_pub;
	geometry_msgs::Twist twist_goal;
};

#endif // FOLLOW_ROTATION_Node_HPP
