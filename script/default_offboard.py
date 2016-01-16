#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
from datetime import datetime

# import utilities
import thread
import math


class vector3:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


current_pose = vector3()
setpoint_pose = vector3()
UAV_state = mavros_msgs.msg.State()



def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def _local_position_callback(topic):
    current_pose.x = topic.pose.position.x
    current_pose.y = topic.pose.position.y
    current_pose.z = topic.pose.position.z

def _setpoint_position_callback(topic):
    pass

def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z


def is_reached(current, setpoint):
    if (abs(current.x-setpoint.pose.position.x) < 0.3 and
        abs(current.y-setpoint.pose.position.y) < 0.3 and
        abs(current.z-setpoint.pose.position.z) < 0.3):
        print "Point reached!"
        return True
    else:
        return False











def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(10)
    mavros.set_namespace('/mavros')


    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),
        mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
        SP.PoseStamped, _local_position_callback)
    # /mavros/setpoint_raw/target_local
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
        mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    # setup publisher
    # /mavros/setpoint/position/local
    setpoint_local_pub =  mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  

    setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )


    # wait for FCU connection
    while(not UAV_state.connected):
        rate.sleep()

    # initialize the setpoint
    setpoint_msg.pose.position.x = 0
    setpoint_msg.pose.position.y = 0
    setpoint_msg.pose.position.z = 1
	
    mavros.command.arming(True)

    # send 100 setpoints before starting
    for i in range(0,100):
        setpoint_local_pub.publish(setpoint_msg)
        rate.sleep()

    set_mode(0,'OFFBOARD')

    last_request = rospy.Time.now()

    step = 0

    while(True):
        if( UAV_state.mode != "OFFBOARD" and
            (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if( set_mode(0,'OFFBOARD').success):
                rospy.INFO("Offboard enabled")
            last_request = rospy.Time.now()
        else:
            if(not UAV_state.armed and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if( set_arming(True).success):
                    rospy.INFO("Vehicle armed")
                last_request = rospy.Time.now()

        if (is_reached(current_pose, setpoint_msg) and UAV_state.armed):
            print "Now we are in step"
            print step
            if (step == 1):
                    _set_pose(setpoint_msg, 0,0,4.1)
                    break
            elif (step == 2):
                    _set_pose(setpoint_msg, 10,10,4.1)
                    break
            elif (step == 3):
                    _set_pose(setpoint_msg, 20,20,10.1)
                    break
            elif (step == 4):
                    _set_pose(setpoint_msg, 0,0,4.1)
                    break
            else:
                while (UAV_state.mode != "AUTO.LAND"):
                    set_mode(0,'AUTO.LAND')
                    rate.sleep()
                return 0
                     # Exit program
            step+=1
            
        setpoint_local_pub.publish(setpoint_msg)

        rate.sleep()
    return 0




if __name__ == '__main__':
    try:
        main()
    finally:
        pass