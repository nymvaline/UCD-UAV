#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <sstream>



int main(int argc, char **argv){
	ros::init(argc, argv, "dart_node_pose");
	ros::NodeHandle NH;
	ros::Publisher pose_publisher = NH.advertise<std_msgs::Int16>("dart_node_pose",10);
	ros::Rate loop_rate(1);

	int count = 0;
	while(ros::ok()){
		std_msgs::Int16 msg;

		msg.data = (int16_t)count;
		ROS_INFO("[DART_NODE_POSE Pub] I publisehd %d\n", msg.data);
		pose_publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		count++;

	}
	return 0;
}
