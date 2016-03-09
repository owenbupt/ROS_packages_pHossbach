#include "c2c_right_node.hpp"

C2C_RIGHT_Node::C2C_RIGHT_Node()
{
	transform_sub = nh.subscribe ("/multi_boards_vrm_" + side + "/transform", 1, &C2C_RIGHT_Node::arsys_transform_callback, this);
	transform_pub = nh.advertise<geometry_msgs::TransformStamped>("/c2c/transform_raw", 100);
	read_robotModel_input();
}


// callback function gets called whenever ar_sys is publishing a new message to .../transform
void C2C_RIGHT_Node::arsys_transform_callback (const geometry_msgs::TransformStamped& transformMsg)
{
	int board_number = transformMsg.child_frame_id[1] - '0';			// finds out the number of the subscribed marker

	//std::cout << "Node running! Time: " << ros::Time::now() << "\n";		// for debugging

	bool run_complete = (((board_number == 0) && (last_entry != -1)) || (last_entry >= board_number));	// true when one run is complete

	if (run_complete)
	{

		//std::cout << "!\n";											// for debugging
		//for (int i = 0; i < count; i++) { std::cout << entry_changed[i] << "_"; } std::cout << "\n";

		int detected_count = 0;
		for (int i = 0; i < count; i++) { if (entry_changed[i]) { detected_count++; } }		// counts the detected boards

		geometry_msgs::TransformStamped transformMsg;			// temporary message for publishing

		switch (detected_count) {

			case 0: break;	// 0 boards detected

			case 1:		// 1 board detected
			{
				for (int i = 0; i < count; i++)
				{
					if (entry_changed[i])			// get the detected board
					{
						tf::StampedTransform stampedTransform;
						tf::transformStampedMsgToTF(collected_msgs[i], stampedTransform);

						tf::transformStampedTFToMsg(stampedTF_filter(steady_transform, get_c2c(stampedTransform)), transformMsg);
						transform_pub.publish(transformMsg);

						//tf_broadcaster.sendTransform(stampedTF_filter(steady_transform, get_c2c(stampedTransform)));
						//std::cout << stampedTF_filter(steady_transform, get_c2c(stampedTransform)).getOrigin().length() << "\n";

						break;
					}
				}
				break;
			}


			case 2:		// 2 boards detected
			{
				for (int i = 0; i < count; i++)				// get the first detected board
				{
					if (entry_changed[i])
					{
						for (int j = i+1; j < count; j++)				// get the second detected board
						{
							if (entry_changed[j])
							{
								if ((j - i) == 1 && i%2 == 0)		// if the two detected boards belong together
								{
									tf::transformStampedTFToMsg(stampedTF_filter(steady_transform, get_c2c(merge_msgsToTF(collected_msgs[i], collected_msgs[j]))), transformMsg);
									transform_pub.publish(transformMsg);
								}
								else			// if the two detected boards don't belong together
								{
									tf::transformStampedMsgToTF(collected_msgs[i], c2c_buffer[0]);
									tf::transformStampedMsgToTF(collected_msgs[j], c2c_buffer[1]);
									c2c_buffer[0] = get_c2c(c2c_buffer[0]);
									c2c_buffer[1] = get_c2c(c2c_buffer[1]);

									tf::transformStampedTFToMsg(stampedTF_filter(steady_transform, choose_c2c()), transformMsg);
									transform_pub.publish(transformMsg);
								}
								break;
							}
						}
						break;
					}
				}
				break;
			}


			case 3:		// 3 boards detected
			{
				for (int i = 0; i < count; i++)				// get the first detected board
				{
					if (entry_changed[i])
					{
						for (int j = i+1; j < count; j++)				// get the second detected board
						{
							if (entry_changed[j])
							{
								for (int k = j+1; k < count; k++)				// get the third detected board
								{
									if (entry_changed[k])
									{
										tf::StampedTransform stampedTransform1;			// temporary stampedTransform

										if ((j - i) == 1 && i%2 == 0)			// if i and j belong together
										{
											tf::transformStampedMsgToTF(collected_msgs[k], stampedTransform1);
											c2c_buffer[1] = get_c2c(stampedTransform1);
											c2c_buffer[0] = get_c2c(merge_msgsToTF(collected_msgs[i], collected_msgs[j]));
										}
										else			// else j and k belong together
										{
											tf::transformStampedMsgToTF(collected_msgs[i], stampedTransform1);
											c2c_buffer[0] = get_c2c(stampedTransform1);
											c2c_buffer[1] = get_c2c(merge_msgsToTF(collected_msgs[j], collected_msgs[k]));
										}

										tf::transformStampedTFToMsg(stampedTF_filter(steady_transform, choose_c2c()), transformMsg);
										transform_pub.publish(transformMsg);

										break;
									}
								}
								break;
							}
						}
						break;
					}
				}
				break;
			}


			case 4:		// 4 boards detected
			{
				int buffer = 0;
				for (int i = 0; i < count; i=i+2)	// get the boards which belong together
				{
					if (entry_changed[i])
					{
						c2c_buffer[buffer] = get_c2c(merge_msgsToTF(collected_msgs[i], collected_msgs[i+1]));	// save each merging to c2c_buffer
						buffer += 1;
					}
				}

				tf::transformStampedTFToMsg(stampedTF_filter(steady_transform, choose_c2c()), transformMsg);
				transform_pub.publish(transformMsg);

				break;
			}

		}

		for (int i = 0; i < count; i++) { entry_changed[i] = false; }		// resetting the entry_changed for the next run
	}

	// Saving the informations of the present detected board
	entry_changed[board_number] = true;
	last_entry = board_number;
	collected_msgs[board_number] = transformMsg;
}


// Calculating and returning the center2center transform which belongs to stampedT_in
// [ A(front), B(right), C(back) or D(left) ]
tf::StampedTransform C2C_RIGHT_Node::get_c2c(const tf::StampedTransform& stampedT_in)
{
	tf::Transform T_in = tf::Transform(stampedT_in.getRotation(), stampedT_in.getOrigin());

	if (stampedT_in.child_frame_id_[1] != '_')		// if stampedT_in was stand-alone (previously 1 transforms)
	{
		switch (stampedT_in.child_frame_id_[1] - '0')
		{
			case 0: case 1:	return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[3].inverse(), ros::Time::now(), "apollon_center", "A_" + side + "_c2c");
			case 2: case 3:	return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[4].inverse()*boreas[3].inverse(), ros::Time::now(), "apollon_center", "B_" + side + "_c2c");
			case 4: case 5:	return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[5].inverse()*boreas[3].inverse(), ros::Time::now(), "apollon_center", "C_" + side + "_c2c");
			case 6: case 7:	return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[6].inverse()*boreas[3].inverse(), ros::Time::now(), "apollon_center", "D_" + side + "_c2c");
		}
	}

	else		// else stampedT_in was merged (previously 2 transforms)
	{
		if (stampedT_in.child_frame_id_[0] == 'A')
			return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[3].inverse(), ros::Time::now(), "apollon_center", "A_" + side + "_c2c");
		else if (stampedT_in.child_frame_id_[0] == 'B')
			return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[4].inverse()*boreas[3].inverse(), ros::Time::now(), "apollon_center", "B_" + side + "_c2c");
		else if (stampedT_in.child_frame_id_[0] == 'C')
			return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[5].inverse()*boreas[3].inverse(), ros::Time::now(), "apollon_center", "C_" + side + "_c2c");
		else if (stampedT_in.child_frame_id_[0] == 'D')
			return tf::StampedTransform(apollon[1]*apollon[2]*T_in*boreas[6].inverse()*boreas[3].inverse(), ros::Time::now(), "apollon_center", "D_" + side + "_c2c");
	}
}


// Merging two boards which belong together (same robotside / same transform)
// msg1 contains transform information about the exterior board (4 small markers)
// msg2 contains transform information about the center board (1 big marker)
// Merging with respect to weight_of_center!
tf::StampedTransform C2C_RIGHT_Node::merge_msgsToTF(const geometry_msgs::TransformStamped& msg1, const geometry_msgs::TransformStamped& msg2)
{
	tf::StampedTransform stampedTransform_1;
	tf::StampedTransform stampedTransform_2;

	tf::transformStampedMsgToTF(msg1, stampedTransform_1);
	tf::transformStampedMsgToTF(msg2, stampedTransform_2);

	tf::Transform t_merged = tf::Transform(stampedTransform_1.getRotation().slerp(stampedTransform_2.getRotation(), weight_of_center),
								tf::Vector3(
								(stampedTransform_1.getOrigin().getX() * (1 - weight_of_center) + stampedTransform_2.getOrigin().getX() * weight_of_center),
								(stampedTransform_1.getOrigin().getY() * (1 - weight_of_center) + stampedTransform_2.getOrigin().getY() * weight_of_center),
								(stampedTransform_1.getOrigin().getZ() * (1 - weight_of_center) + stampedTransform_2.getOrigin().getZ() * weight_of_center) )
								);

	// Returning stampedTransform depending on robotSide [ A(front), B(right), C(back) or D(left) ]
	switch (msg1.child_frame_id[1] - '0') {
		case 0: return tf::StampedTransform(t_merged, ros::Time::now(), msg1.header.frame_id, "A_" + side);
		case 2: return tf::StampedTransform(t_merged, ros::Time::now(), msg1.header.frame_id, "B_" + side);
		case 4: return tf::StampedTransform(t_merged, ros::Time::now(), msg1.header.frame_id, "C_" + side);
		case 6: return tf::StampedTransform(t_merged, ros::Time::now(), msg1.header.frame_id, "D_" + side); }
}


// Choosing and returning best transform from c2c_buffer
// (assuming that A has the richest information, B and D the second richest and C the poorest information)
// [ A(front), B(right), C(back) or D(left) ]
tf::StampedTransform C2C_RIGHT_Node::choose_c2c()
{
	char buffer_0 = c2c_buffer[0].child_frame_id_[0];

	if	(buffer_0 == 'A') return c2c_buffer[0];
	else if (buffer_0 == 'B') return c2c_buffer[0];
	else if (buffer_0 == 'C') return c2c_buffer[1];
}


// Low-pass filter with certain filter_change_rate for filtering the output
// high change_rate	--> faster, less smooth
// low change_rate	--> slower, more smooth
tf::StampedTransform& C2C_RIGHT_Node::stampedTF_filter(tf::StampedTransform &steady, const tf::StampedTransform &fresh)
{
	ros::Duration missingTime(0.2);
	if (steady.stamp_+missingTime < fresh.stamp_) {
		steady.setOrigin(fresh.getOrigin());
		steady.setRotation(fresh.getRotation());
		steady.stamp_ = fresh.stamp_;
		return steady;
	}

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

  // tf broadcasting for debugging
	tf_broadcaster.sendTransform(steady);
	//std::cout << steady.getOrigin().length() << "\n";

	return steady;
}


// read robotino models from .../data/..._setup_2016.txt
void C2C_RIGHT_Node::read_robotModel_input()
{
		float tf_data[14];
		std::ifstream file_A ("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localization/data/apollon_setup_2016.txt");
		std::ifstream file_B ("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localization/data/boreas_setup_2016.txt");

		if ( !file_A.is_open() || !file_B.is_open() )
		{
			std::cout << "One of the Robot Model input files seems to be damaged or does not exist. Required data is missing!\nPlease check the files and start the node again.\n";
			return;
		}

		for(int i = 0; i < 7; i++)
		{
			for(int j = 0; j < 7; j++)
			{
				file_A >> tf_data[j];
				file_B >> tf_data[j+7];
			}
			apollon[i] = tf::Transform(tf::Quaternion(tf_data[0], tf_data[1], tf_data[2], tf_data[3]), tf::Vector3(tf_data[4], tf_data[5], tf_data[6]));
			boreas[i] = tf::Transform(tf::Quaternion(tf_data[7], tf_data[8], tf_data[9], tf_data[10]), tf::Vector3(tf_data[11], tf_data[12], tf_data[13]));
		}
}
