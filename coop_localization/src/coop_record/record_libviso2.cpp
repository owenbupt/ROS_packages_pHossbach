#include "record_libviso2_node.hpp"

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "record_libviso2"); // Name of the node
	RECORD_LIBVISO2_Node Node;

	//int32_t looprate = 10000; //hz
	//ros::Rate loop_rate(looprate);

	ros::spin();

	/*while(Node.nh.ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}*/
}
