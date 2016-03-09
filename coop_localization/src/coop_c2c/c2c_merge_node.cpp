#include "c2c_merge_node.hpp"

C2C_MERGE_Node::C2C_MERGE_Node()
{
	ros::NodeHandle nh("~");
	//nh.param("weight_of_right", weight_of_right, double(0.5));
	//nh.param("filter_change_rate", filter_change_rate, double(0.2));
	c2c_sub = nh.subscribe ("/c2c/transform_raw", 1, &C2C_MERGE_Node::c2c_callback, this);		// Subscribes to "/c2c" which is published by c2c_left and c2c_right
	c2c_merged_pub = nh.advertise<geometry_msgs::TransformStamped>("/c2c/transform", 100);	// Publishes merged transform to "c2c_merged"
}

void C2C_MERGE_Node::c2c_callback (const geometry_msgs::TransformStamped& c2c_transformMsg)
{
	ros::Duration missingTime(0.2);
	tf::StampedTransform stampedTransform_fresh;
	tf::transformStampedMsgToTF(c2c_transformMsg, stampedTransform_fresh);

	char side = stampedTransform_fresh.child_frame_id_[4];
	if (side == 'l') c2c_in[0]=stampedTransform_fresh;
	else if (side == 'r') c2c_in[1]=stampedTransform_fresh;

// actual merging happens here
		bool test = steady_transform.stamp_==ros::Time(0.0);
		std::cout << steady_transform.stamp_ << " fresh_X " << stampedTransform_fresh.getOrigin().getX() << "\n";

		if (steady_transform.stamp_+ros::Duration(0.2) <= ros::Time::now() && !reseted1)
			{std::cout << "RESET1" << "\n"; reseted1=true; return;}
		else if (steady_transform.stamp_+ros::Duration(0.2) <= ros::Time::now() && !reseted2)
			{std::cout << "RESET2" << "\n"; reseted2=true; return;}
  	else if (steady_transform.stamp_+ros::Duration(0.2) <= ros::Time::now() && reseted1 && reseted2)
			{ c2c_in[0]=stampedTransform_fresh;
				c2c_in[1]=stampedTransform_fresh;
				stampedTransform_fresh = stampedTF_merge(c2c_in[0], c2c_in[1]);
				std::cout << "NEWWWWW" << stampedTransform_fresh.getOrigin().getX() << "\n"; reseted1=false; reseted2=false;}
		else if (c2c_in[0].stamp_==ros::Time(0.0) || c2c_in[0].stamp_+ros::Duration(0.2) <= ros::Time::now())
			{stampedTransform_fresh = c2c_in[1]; std::cout << "take c2c_in1" << "\n";}
		else if (c2c_in[1].stamp_==ros::Time(0.0) || c2c_in[1].stamp_+ros::Duration(0.2) <= ros::Time::now())
			{stampedTransform_fresh = c2c_in[0]; std::cout << "take c2c_in0" << "\n";}
		else
			{stampedTransform_fresh = stampedTF_merge(c2c_in[0], c2c_in[1]); std::cout << "merged" << "\n";}


		steady_transform = stampedTF_filter(steady_transform, stampedTransform_fresh);




// converting the message and publishing to "/c2c_merged"

		geometry_msgs::TransformStamped transformMsg;
		tf::transformStampedTFToMsg(steady_transform, transformMsg);
		c2c_merged_pub.publish(transformMsg);


// broadcasting for rviz and screen output for debugging

		tf_broadcaster.sendTransform(steady_transform);

		//std::ofstream file("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localisation/data/results/example.txt", std::ios::app);

		//if ( !file.is_open())
		//{ std::cout << "Output file seems to be damaged or does not exist. Required data is missing!\nPlease check the file and start the node again.\n";
			//return; }

		//file << steady_transform.getOrigin().getX() << " " << steady_transform.getOrigin().getZ() << "\n";

  	//file.close();

		std::cout << "center2center = " << round(steady_transform.getOrigin().length()*1000) << " mm   (Txyz = "
					<< round(steady_transform.getOrigin().getX()*1000) << " "
					<< round(steady_transform.getOrigin().getY()*1000) << " "
					<< round(steady_transform.getOrigin().getZ()*1000) << " "
					<< steady_transform.getRotation().getAngle() <<
					")\n\n";

}

tf::StampedTransform C2C_MERGE_Node::stampedTF_merge(tf::StampedTransform transform1, tf::StampedTransform transform2)
{
	tf::StampedTransform merge;
	tf::Vector3 pos_1 = transform1.getOrigin();
	tf::Vector3 pos_2 = transform2.getOrigin();
	tf::Vector3 pos
	(
		(1 - weight_of_right) * pos_1.x() + weight_of_right * pos_2.x(),
		(1 - weight_of_right) * pos_1.y() + weight_of_right * pos_2.y(),
		(1 - weight_of_right) * pos_1.z() + weight_of_right * pos_2.z()
	);
	merge.setOrigin(pos);

	tf::Quaternion Q_1 = transform1.getRotation();
	tf::Quaternion Q_2 = transform2.getRotation();
	tf::Quaternion Q;

    // Interpolation between two quaternions.
	Q = Q_1.slerp(Q_2, weight_of_right);
	merge.setRotation(Q);

	merge.stamp_ = ros::Time::now();
	merge.child_frame_id_ = "c2c";
	merge.frame_id_ = "a_center";

	return merge;
}


tf::StampedTransform& C2C_MERGE_Node::stampedTF_filter(tf::StampedTransform &steady, const tf::StampedTransform &fresh)
{
	if (steady.stamp_==ros::Time(0.0)) {
		steady.setOrigin(fresh.getOrigin());
		steady.setRotation(fresh.getRotation());
		steady.stamp_ = ros::Time::now();
		return steady;
	}

	/*else if(steady.stamp_+ros::Duration(0.2) <= ros::Time::now()) {
		ros::Time zeit = steady.stamp_+ros::Duration(0.2);
		std::cout << "NEU! " << fresh.getOrigin().getX() << "\n";
			steady.setOrigin(fresh.getOrigin());
			steady.setRotation(fresh.getRotation());
			steady.stamp_ = ros::Time::now();
		return steady;
	}*/

	else {
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

	steady.stamp_ = ros::Time::now();

	return steady;
  }
}
