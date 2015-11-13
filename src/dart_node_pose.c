#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "dart_node_pose");
	ros::NodeHandle NH;
	ros::Publisher pose_publisher = NH.advertise<std_msgs::String>("dart_node_pose",10);
	ros::Rate loop_rate(1);

	int count = 0;
	while(ros::ok()){
		std_msgs::String msg;

		std::stringstream ss;
		ss<<"hellow world " <<count;
		msg.data = ss.str();
		pose_publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		count++;

	}
	return 0;
}
