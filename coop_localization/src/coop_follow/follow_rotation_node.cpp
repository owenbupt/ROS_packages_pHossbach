#include "follow_rotation_node.hpp"

FOLLOW_ROTATION_Node::FOLLOW_ROTATION_Node()
{
	c2c_sub = nh.subscribe ("/c2c", 1, &FOLLOW_ROTATION_Node::c2c_callback, this);		// Subscribes to "/c2c_merged" which is published by c2c_merge_node
	follow_rotation_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void FOLLOW_ROTATION_Node::c2c_callback (const geometry_msgs::TransformStamped& c2c_transformMsg)
{
		double distance = 1.5;

		tf::StampedTransform stampedTransform;
		tf::transformStampedMsgToTF(c2c_transformMsg, stampedTransform);

		tf::Matrix3x3 mat(stampedTransform.getRotation());
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);

		std::cout << "Translation X: " << stampedTransform.getOrigin().getX() << "\n";
		std::cout << "Rotation ROLL:" << roll << "\n";
		std::cout << "Rotation PITCH:" << pitch << " abs: " << std::abs(pitch) << "\n";
		std::cout << "Rotation YAW:" << yaw << "\n";


		twist_goal.linear.x = twist_goal.linear.y = twist_goal.linear.z = 0;
		twist_goal.angular.x = twist_goal.angular.y = twist_goal.angular.z = 0;



		if (std::abs(roll) > 0.5 || std::abs(yaw) > 0.5) {
			if(pitch < 0) twist_goal.angular.z = (3.15-std::abs(pitch));
			else if(pitch > 0) twist_goal.angular.z = -(3.15-std::abs(pitch));
		}

		else if (std::abs(pitch) > 0.5) {
			twist_goal.angular.z = -pitch;
		}

		else
		{
			if (stampedTransform.getOrigin().getX() < -0.008) {twist_goal.linear.y = -stampedTransform.getOrigin().getX();}
			else if (stampedTransform.getOrigin().getX() > 0.008) {twist_goal.linear.y = -stampedTransform.getOrigin().getX();}
			else {twist_goal.linear.y = 0.0;}

			if (stampedTransform.getOrigin().getZ() < (distance-0.02) ) {twist_goal.linear.x = (distance - stampedTransform.getOrigin().getZ())*0.8;}
			else if (stampedTransform.getOrigin().getZ() > (distance+0.02)) {twist_goal.linear.x = -(stampedTransform.getOrigin().getZ() - distance)*0.8;}
			else {twist_goal.linear.x = 0.0;}

			if (pitch < -0.02) {twist_goal.angular.z = std::abs(pitch);}
			else if (pitch > 0.02) {twist_goal.angular.z = -std::abs(pitch);}
			else {twist_goal.angular.z = 0.0;}
		}






		follow_rotation_pub.publish(twist_goal);

		//std::cout << "\ncenter2center = " << round(steady_transform.getOrigin().length()*1000) << " mm   (Txyz = "
					//<< round(steady_transform.getOrigin().getX()*1000) << " "
					//<< round(steady_transform.getOrigin().getY()*1000) << " "
					//<< round(steady_transform.getOrigin().getZ()*1000) << ")\n";

}
