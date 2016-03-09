#ifndef RECORD_WEBCAM2_Node_HPP
#define RECORD_WEBCAM2_Node_HPP

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"

using std::string;


class RECORD_WEBCAM2_Node
{
public:
	RECORD_WEBCAM2_Node();
	ros::NodeHandle nh;

	void transform_callback(const geometry_msgs::TransformStamped& transformMsg);	// ROS callback

private:
	ros::Subscriber webcam2_sub;
	std::ofstream file;
	string dateString = "", tmp = "";

};

#endif // RECORD_WEBCAM2_Node_HPP
