/*
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// By hengjiu kang in UCD.DART for UCD.UAV project
// This is UAVxROS test program. This is aiming to avoide collision to ground or to
// some specific target. This program is feasibility test for collision avoidance.
// This program will 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped set_point;
std_msgs::Float32 radar_distance;


bool is_reached(geometry_msgs::PoseStamped& pose,
                geometry_msgs::PoseStamped& setpoint){
	// Fake reached check. Count 5s (5s*20Hz)
	// if (ros::Time::now() - _fake_timer > ros::Duration(5.0)){
		// _fake_timer = ros::Time::now();
		// return 1;
	// }else{
		// return 0;
	// }
    // When approach to within 0.5 m
    if (abs(pose.pose.position.x - setpoint.pose.position.x)<0.5 &&
        abs(pose.pose.position.y - setpoint.pose.position.y)<0.5 &&
        abs(pose.pose.position.z - setpoint.pose.position.z)<0.5){
        ROS_INFO("Reached point!");
        return true;
    }else{
        return false;
    }

	return false;
}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose.pose.position.x = msg->pose.position.x;
    current_pose.pose.position.y = msg->pose.position.y;
    current_pose.pose.position.z = msg->pose.position.z;
}

void radar_cb(const std_msgs::Float32::ConstPtr& msg){
	radar_distance.data = msg->data;
}

void _set_pose(geometry_msgs::PoseStamped& pose, double x, double y, double z){
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

}




int main(int argc, char** argv){
	ros::init(argc, argv, "Sonar_collision_avoidance");
	ros::NodeHandle nh;

	current_pose.pose.position.x = 0.0;
	current_pose.pose.position.y = 0.0;
	current_pose.pose.position.z = 0.0;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, current_pose_cb);
    ros::Subscriber radar_sub = nh.subscribe<std_msgs::Float32>
    		("/dart_radar_distance", 10, radar_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate(20.0);

    
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }



    _set_pose(set_point, 0,0,5);


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(set_point);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;





    // first make the sender as 'controller' to control how high it flies.
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (is_reached(current_pose, set_point) && current_state.armed){
            _set_pose(set_point, 0, 0, radar_distance.data+10.0);
        }

        local_pos_pub.publish(set_point);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}