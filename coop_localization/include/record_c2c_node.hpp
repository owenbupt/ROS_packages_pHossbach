#ifndef RECORD_C2C_Node_HPP
#define RECORD_C2C_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"

using std::string;


class RECORD_C2C_Node
{
public:
	RECORD_C2C_Node();
	ros::NodeHandle nh;

	void transform_callback(const geometry_msgs::TransformStamped& transformMsg);	// ROS callback

private:
	ros::Subscriber c2c_sub;
	std::ofstream file;
	string dateString = "", tmp = "";

};

#endif // RECORD_C2C_Node_HPP
