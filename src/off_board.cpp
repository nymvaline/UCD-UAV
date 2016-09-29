
/*
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
ros::Time _fake_timer;


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

void _set_pose(geometry_msgs::PoseStamped& pose, double x, double y, double z){
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Initialize for current_pose
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.0;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, current_pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 2;
    pose.pose.position.y = 2;
    pose.pose.position.z = 5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    int step=1;
    _fake_timer = ros::Time::now();

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

        if (is_reached(current_pose, pose) && current_state.armed){

        	switch(step){
        		case 1:
        			_set_pose(pose, 0,0,5);
			        break;
			    case 2:
			    	_set_pose(pose, 0, 50,5);
			    	break;
			    case 3:
			    	_set_pose(pose, 20, 20, 5);
			    	break;
			    case 4:
			    	_set_pose(pose, 20,0,10);
			    	break;
			    case 5:
			    	_set_pose(pose, 0,0,10);
			    	break;
			    default:
                        while (current_state.mode != "AUTO.LAND"){
					    offb_set_mode.request.custom_mode= "AUTO.LAND";
				    	set_mode_client.call(offb_set_mode);
                        ros::spinOnce();
                        rate.sleep();
                        }

                        return 0;
			    	// Exit program

			    
        	}
        	step++;
        	ROS_INFO("Now we are at step: %d\n", step);
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




// #include <ros/ros.h>
// #include <std_msgs/String.h> 
// #include <stdio.h>
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/Vector3Stamped.h"
// //#include </home/mahesh/catkin_ws/src/beginner_tutorials/src/Qualisys.h>

// int main(int argc, char **argv)
// {
//    ros::init(argc, argv, "data_to_setpoint_4");
//    ros::NodeHandle n;

//    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
//    ros::Rate loop_rate(10);

//    geometry_msgs::PoseStamped msg;
//    int count = 1;
   
  

  
//     //PositionReciever qp;
//     //Body some_object;
//     //qp.connect_to_server();

  
//    while(ros::ok()){
//      //some_object = qp.getStatus();
//     // some_object.print();
//     //printf("%f\n",some_object.position_x);
//        msg.header.stamp = ros::Time::now();
//        msg.header.seq=count;
//        msg.header.frame_id = 1;
//        msg.pose.position.x = 0.0;//0.001*some_object.position_x;
//        msg.pose.position.y = 0.0;//0.001*some_object.position_y;
//        msg.pose.position.z =0.6;//0.001*some_object.position_z;
//        msg.pose.orientation.x = 0;
//        msg.pose.orientation.y = 0;
//        msg.pose.orientation.z = 0;
//        msg.pose.orientation.w = 1;

//        chatter_pub.publish(msg);
//        ros::spinOnce();
//        count++;
//        loop_rate.sleep();
//    }
   
      
//    return 0;
// }



// // #include "ros/ros.h"
// // #include "geometry_msgs/PoseStamped.h"
// // #include "mavros/mavros.h"
// // #include "std_msgs/Header.h"


// // ros::Publisher mavros_pos_pub;
// // geometry_msgs::PoseStamped pos_msg;
// // std_msgs::Header header_info;

// // void pos_sub_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

// // void pos_sub_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
// //   // ROS_INFO("[Listener] The position is: [%d] [%d] [%s]", header_info.seq, header_info.stamp, header_info.frame_id);
// //   ROS_INFO("[Listener] The position.x is [%lf]", msg->pose.position.x);
// //   // pos_msg.header = msg->header;
// //   // pos_msg.pose = msg->pose;
// // }

// // int main(int argc, char **argv)
// // {

// //   ros::init(argc, argv, "node");
// //   ros::NodeHandle nh;

// //   mavros_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1000);
// //   ros::Subscriber pos_subscriber = nh.subscribe("/mavros/local_position/local", 1000, pos_sub_Callback);

// //   ros::Rate loop_rate(10.0);

// //   for (;;){
// //     pos_msg.header.stamp = ros::Time::now();
// //     pos_msg.pose.position.x += 0.1;
// //     mavros_pos_pub.publish(pos_msg);
// //     ros::spinOnce();
// //     loop_rate.sleep();
// //   }

// //   ros::spin();
// //   // loop_rate.sleep();
// //   return 0;
// // }






// //   ros::ServiceClient mavros_set_mode_client = nh.serviceClient<mavros::SetMode>("mavros/set_mode");
// //   mavros::SetMode set_mode;
// //   set_mode.request.custom_mode = "OFFBOARD";

// //   ros::ServiceClient mavros_nav_guided_client = nh.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
// //   mavros::CommandBool nav_guided;
// //   nav_guided.request.value = true;

// //   bool offboard_commands_enabled = false;
// //   bool nav_guided_enabled = false;

// //   ros::Rate loop_rate(100.0);

// //   while(ros::ok())
// //   {
// //     if (!offboard_commands_enabled) {
// //       if (mavros_set_mode_client.call(set_mode))
// //       {
// //         ROS_INFO("Set mode: OFFBOARD enabled!");
// //       }
// //       else
// //       {
// //         ROS_INFO("Offboard mode still not enabled!");
// //       }
// //     }

// //     // Write desired setpoint value to fmu_controller_setpoint varialbe.

// //     mavros_control_pub.publish(fmu_controller_setpoint);

// //     if(!nav_guided_enabled)
// //     {
// //       if (mavros_nav_guided_client.call(pos_msg))
// //       {
// //     ROS_INFO("Nav guided: OFFBOARD enabled!");
// //       }
// //     }

// //     ros::spinOnce();

// //     loop_rate.sleep();
// //   }

// // }

