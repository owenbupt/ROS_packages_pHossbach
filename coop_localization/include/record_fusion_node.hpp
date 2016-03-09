#ifndef RECORD_FUSION_Node_HPP
#define RECORD_FUSION_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"

using std::string;


class RECORD_FUSION_Node
{
public:
	RECORD_FUSION_Node();
	ros::NodeHandle nh;

	void transform_callback(const geometry_msgs::TransformStamped& transformMsg);	// ROS callback

private:
	ros::Subscriber fusion_sub;
	std::ofstream file;
	string dateString = "", tmp = "";

};

#endif // RECORD_FUSION_Node_HPP
