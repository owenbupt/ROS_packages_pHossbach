#include "c2c_merge_node.hpp"

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "c2c_merge"); // Name of the node
	C2C_MERGE_Node Node;

	//int32_t looprate = 10000; //hz
	//ros::Rate loop_rate(looprate);

	ros::spin();

	/*while(Node.nh.ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}*/
}
