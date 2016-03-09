#include "c2c_right_node.hpp"

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "c2c_right"); // Name of the node
	C2C_RIGHT_Node Node;

	//int32_t looprate = 10000; //hz
	//ros::Rate loop_rate(looprate);

	ros::spin();

	/*while(Node.nh.ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}*/
}
