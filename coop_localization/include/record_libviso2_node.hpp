#ifndef RECORD_LIBVISO2_Node_HPP
#define RECORD_LIBVISO2_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"

using std::string;


class RECORD_LIBVISO2_Node
{
public:
	RECORD_LIBVISO2_Node();
	ros::NodeHandle nh;

	void pose_callback(const geometry_msgs::PoseStamped& poseMsg);	// ROS callback

private:
	ros::Subscriber libviso2_sub;
	std::ofstream file;
	string dateString = "", tmp = "";

};

#endif // RECORD_LIBVISO2_Node_HPP
