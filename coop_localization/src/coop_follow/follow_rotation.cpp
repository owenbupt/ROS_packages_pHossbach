#include "follow_rotation_node.hpp"

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "follow_rotation"); // Name of the node
	FOLLOW_ROTATION_Node Node;

	//int32_t looprate = 10000; //hz
	//ros::Rate loop_rate(looprate);

	ros::spin();

	/*while(Node.nh.ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}*/
}
