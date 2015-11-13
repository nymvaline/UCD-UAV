#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Temperature.h"

ros::Subscriber temp_subscriber;

// Topic messages callback
void tempCallback(const sensor_msgs::Temperature::ConstPtr& msg)
{
	ROS_INFO("[Linstener] I heard: [%.2f] raw temperature value!", msg->temperature);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "temp_listeneri_node");
	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("/mavros/imu/temperature", 1000, tempCallback);

	ros::spin();
	return 0;
}
