/*
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
