#include "record_fusion_node.hpp"

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "record_fusion"); // Name of the node
	RECORD_FUSION_Node Node;

	//int32_t looprate = 10000; //hz
	//ros::Rate loop_rate(looprate);

	ros::spin();

	/*while(Node.nh.ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}*/
}
