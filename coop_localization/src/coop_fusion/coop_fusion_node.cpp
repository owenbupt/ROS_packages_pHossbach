#include "coop_fusion_node.hpp"

COOP_FUSION_Node::COOP_FUSION_Node()
{
	c2c_sub = nh.subscribe ("/c2c/transform", 1, &COOP_FUSION_Node::c2c_callback, this);		// Subscribes to "/c2c" which is published by c2c_left and c2c_right
	libViso2_sub = nh.subscribe ("/stereo_odometer/pose", 1, &COOP_FUSION_Node::libViso2_callback, this);		// Subscribes to "/c2c" which is published by c2c_left and c2c_right
	fusion_pub = nh.advertise<geometry_msgs::TransformStamped>("/fusion/transform", 100);	// Publishes merged transform to "c2c_merged"
	odom_reset_pub = nh.advertise<geometry_msgs::PoseStamped>("/stereo_odometer/pose", 100);	// Publishes merged transform to "c2c_merged"
}



void COOP_FUSION_Node::c2c_callback (const geometry_msgs::TransformStamped& c2c_transformMsg)
{
	tf::StampedTransform c2c_fresh;
	tf::transformStampedMsgToTF(c2c_transformMsg, c2c_fresh);

	c2c_fresh = tf::StampedTransform(tf::Transform(tf::Quaternion(c2c_fresh.getRotation().getX(),c2c_fresh.getRotation().getY(),c2c_fresh.getRotation().getZ(),c2c_fresh.getRotation().getW()),
							tf::Vector3(c2c_fresh.getOrigin().getX() - 2.4, 0, c2c_fresh.getOrigin().getZ() + 1)), ros::Time::now(), "GLOBAL_W", "c2c_fresh");

	memory[0] = c2c_fresh;

	tf_broadcaster.sendTransform(c2c_fresh);


	tf::StampedTransform fusion;

	if (memory[1].getOrigin().getX() != 0 && memory[0].getOrigin().getX() != 0) {
			fusion = stampedTF_fusion(memory[0], memory[1]);
	}

	geometry_msgs::TransformStamped transformMsg;
	tf::transformStampedTFToMsg(fusion, transformMsg);
	fusion_pub.publish(transformMsg);


  // broadcasting for rviz and screen output for debugging
	tf_broadcaster.sendTransform(fusion);

	std::cout << "got c2c at" << c2c_fresh.stamp_ << "\n";
}



void COOP_FUSION_Node::libViso2_callback (const geometry_msgs::PoseStamped& libViso2_poseMsg)
{
	q0.setEuler(-M_PI/2, 0, 0);
	ros::Duration missingTime(0.2);

	tf::StampedTransform libViso2_fresh =
			tf::StampedTransform(tf::Transform(q0,tf::Vector3(-2.4+odom_correction[4],0,2.4+odom_correction[5])) * tf::Transform(tf::Quaternion(libViso2_poseMsg.pose.orientation.x, libViso2_poseMsg.pose.orientation.y, libViso2_poseMsg.pose.orientation.z, libViso2_poseMsg.pose.orientation.w),
			tf::Vector3(libViso2_poseMsg.pose.position.x, 0, libViso2_poseMsg.pose.position.z)), /*libViso2_poseMsg.header.stamp*/ ros::Time::now(), "GLOBAL_W", "libViso2_fresh");

	memory[1] = libViso2_fresh;

	tf_broadcaster.sendTransform(libViso2_fresh);

	tf::StampedTransform fusion;

	if (memory[1].getOrigin().getX() != 0 && memory[0].getOrigin().getX() != 0) {
				if (memory[0].stamp_+missingTime < memory[1].stamp_) {
						fusion = stampedTF_fusion(memory[1], memory[1]);
						std::cout << "c2c verloren bei: " << memory[1].stamp_ << "\n";
						if (!odom_corrected) {
								odom_correction[4] += memory[0].getOrigin().getX() - memory[1].getOrigin().getX();
								odom_correction[5] += memory[0].getOrigin().getZ() - memory[1].getOrigin().getZ();
								odom_corrected = true;
						}
				}
				else {
						fusion = stampedTF_fusion(memory[0], memory[1]);
						odom_corrected = false;
				}
	}

	geometry_msgs::TransformStamped transformMsg;
	tf::transformStampedTFToMsg(fusion, transformMsg);
	fusion_pub.publish(transformMsg);


  // broadcasting for rviz and screen output for debugging
	tf_broadcaster.sendTransform(fusion);


	/*if (libViso2_fresh.getOrigin().getX() < -22.5 && !odom_corrected) {

		//odom_correction[0] = libViso2_fresh.getRotation().getX();
		//odom_correction[1] = libViso2_fresh.getRotation().getY();
		//odom_correction[2] = libViso2_fresh.getRotation().getZ();
		//odom_correction[3] = libViso2_fresh.getRotation().getW();

		odom_correction[4] += memory[0].getOrigin().getX() - memory[1].getOrigin().getX();
		odom_correction[5] += memory[0].getOrigin().getZ() - memory[1].getOrigin().getZ();

		odom_corrected = true;
	}

	if (libViso2_fresh.getOrigin().getX() > -22.5 && odom_corrected) {
			odom_corrected = false;
	}*/

	std::cout << "got libViso2 at" << libViso2_fresh.stamp_ << "\n";
}




/*tf::StampedTransform& COOP_FUSION_Node::stampedTF_filter(tf::StampedTransform &steady, const tf::StampedTransform &fresh)
{
	tf::Vector3 pos_steady = steady.getOrigin();
	tf::Vector3 pos_fresh = fresh.getOrigin();
	tf::Vector3 pos
	(
		(1 - filter_change_rate) * pos_steady.x() + filter_change_rate * pos_fresh.x(),
		(1 - filter_change_rate) * pos_steady.y() + filter_change_rate * pos_fresh.y(),
		(1 - filter_change_rate) * pos_steady.z() + filter_change_rate * pos_fresh.z()
	);
	steady.setOrigin(pos);

    // Raul modified this in order to avoid singularities in angles
	tf::Quaternion Q_steady = steady.getRotation();
	tf::Quaternion Q_fresh = fresh.getRotation();
	tf::Quaternion Q;

    // Interpolation between two quaternions.
	Q = Q_steady.slerp(Q_fresh, filter_change_rate);
	steady.setRotation(Q);

	steady.stamp_ = fresh.stamp_;

	return steady;
}*/


//transform1=c2c, transform2=libViso2
tf::StampedTransform COOP_FUSION_Node::stampedTF_fusion(tf::StampedTransform transform1, tf::StampedTransform transform2)
{
	tf::StampedTransform fusion;
	tf::Vector3 pos_1 = transform1.getOrigin();
	tf::Vector3 pos_2 = transform2.getOrigin();
	tf::Vector3 pos
	(
		(1 - weight_of_c2c) * pos_2.x() + weight_of_c2c * pos_1.x(),
		(1 - weight_of_c2c) * pos_2.y() + weight_of_c2c * pos_1.y(),
		(1 - weight_of_c2c) * pos_2.z() + weight_of_c2c * pos_1.z()
	);
	fusion.setOrigin(pos);

	tf::Quaternion Q_1 = transform1.getRotation();
	tf::Quaternion Q_2 = transform2.getRotation();
	tf::Quaternion Q;

    // Interpolation between two quaternions.
	Q = Q_2.slerp(Q_1, weight_of_c2c);
	fusion.setRotation(Q);

	fusion.stamp_ = ros::Time::now();
	fusion.frame_id_ = "GLOBAL_W";
	fusion.child_frame_id_ = "fusion";

	return fusion;
}
