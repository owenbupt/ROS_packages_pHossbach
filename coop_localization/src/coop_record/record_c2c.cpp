#include "record_c2c_node.hpp"

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "record_c2c"); // Name of the node
	RECORD_C2C_Node Node;

	//int32_t looprate = 10000; //hz
	//ros::Rate loop_rate(looprate);

	ros::spin();

	/*while(Node.nh.ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}*/
}
