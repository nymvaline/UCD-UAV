#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/*
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
