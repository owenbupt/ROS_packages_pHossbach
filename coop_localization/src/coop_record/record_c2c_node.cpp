#include "record_c2c_node.hpp"

RECORD_C2C_Node::RECORD_C2C_Node()
{
	c2c_sub = nh.subscribe ("/c2c/transform", 1, &RECORD_C2C_Node::transform_callback, this);

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

void RECORD_C2C_Node::transform_callback (const geometry_msgs::TransformStamped& transformMsg)
{

		tf::StampedTransform stampedTransform;
		tf::transformStampedMsgToTF(transformMsg, stampedTransform);

		//std::ofstream file("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localisation/data/results/record_c2c_" + testName + ".txt", std::ios::app);

		file.open("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localisation/data/results/record_" + dateString + "_c2c.txt", std::ios::app);

		//if ( !file.is_open())
		//{ std::cout << "Output file seems to be damaged or does not exist. Required data is missing!\nPlease check the file and start the node again.\n";
			//return; }

		file << stampedTransform.getOrigin().getX()*(-1) << " " << stampedTransform.getOrigin().getZ() << " " << stampedTransform.getOrigin().getY()
		<< " " << stampedTransform.getRotation().getX() << " " << stampedTransform.getRotation().getY() << " " << stampedTransform.getRotation().getZ() << " " << stampedTransform.getRotation().getW()<< "\n";

  	file.close();

}
