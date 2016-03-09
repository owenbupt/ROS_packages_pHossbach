#include "record_libviso2_node.hpp"

RECORD_LIBVISO2_Node::RECORD_LIBVISO2_Node()
{
	libviso2_sub = nh.subscribe ("/stereo_odometer/pose", 1, &RECORD_LIBVISO2_Node::pose_callback, this);

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

void RECORD_LIBVISO2_Node::pose_callback (const geometry_msgs::PoseStamped& poseMsg)
{

		file.open("/local/users/roboterlab/catkin_ws/src/packages_phillip/coop_localisation/data/results/record_" + dateString + "_vo.txt", std::ios::app);

		if ( !file.is_open())
		{ std::cout << "Output file seems to be damaged or does not exist. Required data is missing!\nPlease check the file and start the node again.\n";
			return; }

		file << poseMsg.pose.position.z << " " << poseMsg.pose.position.x << " " << poseMsg.pose.position.y
		<< " " << poseMsg.pose.orientation.x << " " << poseMsg.pose.orientation.y << " " << poseMsg.pose.orientation.z << " " << poseMsg.pose.orientation.w << "\n";

  	file.close();

}
