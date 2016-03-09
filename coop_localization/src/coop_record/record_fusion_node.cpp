#include "record_fusion_node.hpp"

RECORD_FUSION_Node::RECORD_FUSION_Node()
{
	fusion_sub = nh.subscribe ("/fusion/transform", 1, &RECORD_FUSION_Node::transform_callback, this);

			time_t now = time(0);
		  tm *ltm = localtime(&now);

	    tmp = std::to_string(ltm->tm_mday);
	    if (tmp.length() == 1)
	        tmp.insert(0, "0");
	    dateString += tmp;
	    dateString += "-";
	    tmp = std::to_string(1 + ltm->tm_mon);
	    if (tmp.length() == 1)
	        tmp.insert(0, "0");
	    dateString += tmp;
	    dateString += "-";
	    tmp = std::to_string(1900 + ltm->tm_year);
	    dateString += tmp;
	    dateString += " ";
	    tmp = std::to_string(ltm->tm_hour);
	    if (tmp.length() == 1)
	        tmp.insert(0, "0");
	    dateString += tmp;
	    dateString += ":";
	    tmp = std::to_string(1 + ltm->tm_min);
	    if (tmp.length() == 1)
	        tmp.insert(0, "0");
	    dateString += tmp;
  		dateString += ":";
    	tmp = std::to_string(1 + ltm->tm_sec);
    		if (tmp.length() == 1)
        	tmp.insert(0, "0");
    	dateString += tmp;

}

void RECORD_FUSION_Node::transform_callback (const geometry_msgs::TransformStamped& transformMsg)
{

		tf::StampedTransform stampedTransform;
		tf::transformStampedMsgToTF(transformMsg, stampedTransform);

		//std::ofstream file("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localisation/data/results/record_fusion_" + testName + ".txt", std::ios::app);

		file.open("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localisation/data/results/record_" + dateString + "_fusion.txt", std::ios::app);

		//if ( !file.is_open())
		//{ std::cout << "Output file seems to be damaged or does not exist. Required data is missing!\nPlease check the file and start the node again.\n";
			//return; }


		double roll, pitch, yaw;
		tf::Quaternion Q_yaw, Q_roll;

		tf::Matrix3x3(stampedTransform.getRotation()).getRPY(roll, pitch, yaw);

		/*

		std::cout << roll*180/M_PI << " " << pitch*180/M_PI << " " << yaw*180/M_PI << "\n\n";


		Q_yaw.setRPY(0, 0, yaw);
		Q_roll.setRPY((-roll), 0, 0);

		tf::Matrix3x3(Q_yaw).getRPY(roll, pitch, yaw);
		std::cout << roll*180/M_PI << " " << pitch*180/M_PI << " " << yaw*180/M_PI << "\n";

		tf::Matrix3x3(Q_roll).getRPY(roll, pitch, yaw);
		std::cout << roll*180/M_PI << " " << pitch*180/M_PI << " " << yaw*180/M_PI << "\n\n";


		tf::Quaternion Q_corrected = Q_yaw.inverse() * stampedTransform.getRotation() * Q_roll;

		tf::Matrix3x3(Q_corrected).getRPY(roll, pitch, yaw);

		std::cout << roll*180/M_PI << " " << pitch*180/M_PI << " " << yaw*180/M_PI << "\n\n";*/

		file << stampedTransform.getOrigin().getX()*(-1) << " " << stampedTransform.getOrigin().getZ() << " " << stampedTransform.getOrigin().getY()
		<< " " << stampedTransform.getRotation().getX() << " " << stampedTransform.getRotation().getY() << " " << stampedTransform.getRotation().getZ() << " " << stampedTransform.getRotation().getW()
		<< " " << (stampedTransform.getRotation().getAngle()*180/M_PI)
		<< " " << roll*180/M_PI << " " << pitch*180/M_PI << " " << yaw*180/M_PI << "\n";

  	file.close();

}
