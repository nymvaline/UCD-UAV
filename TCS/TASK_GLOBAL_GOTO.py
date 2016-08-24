#!/usr/bin/env python
'''
This is the TASK_GLOBAL_GOTO
This task will guide the drone to the destination point 
by keeping sending setpoint_local
This task is able to check is_reached
once the UAV reached the point, the task will exit
TODO:
implement an appropriate platfrom recognition
implement task publisher that talking to main TCS process
that this TASK has finished its job.

For simplify purpose, 
latitude : x
longitude: y
altitude: z


'''

# import utilities
import math
import sys
import signal
import subprocess
import os
import platform
if (platform.uname()[1]=='ubuntu'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='edison'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='xiaoguang'):
    # this is workstation in 079 Lab
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg
import time
from datetime import datetime

import TCS_util

# Declare some global variables.
# current position
current_position = TCS_util.vector3()
# setpoint message. The type will be changed later in main()
setpoint_msg = 0
# setpoint position
setpoint_position = TCS_util.vector3()
raw_setpoint_position = TCS_util.vector3()
# precision setup. normally set it to 0.5m
precision = 0.000005
# setup frame_id
frame_id='GLOBAL_GOTO'

def set_target(pose, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of 
    mavros.setpoint.PoseStamped
    """
    pose.latitude = x
    pose.longitude = y
    pose.altitude = z
    pose.header=mavros.setpoint.Header(
        frame_id="global_GPS",
        stamp=rospy.Time.now())
    pose.coordinate_frame = pose.FRAME_GLOBAL_TERRAIN_ALT
    pose.type_mask = pose.IGNORE_VX +pose.IGNORE_VY +pose.IGNORE_VZ +pose.IGNORE_AFX+pose.IGNORE_AFY+pose.IGNORE_AFZ+pose.IGNORE_YAW+pose.IGNORE_YAW_RATE
    # pose.velocity.x=10
    # pose.velocity.y=10
    # pose.velocity.z=10
    # pose.acceleration_or_force.x=2
    # pose.acceleration_or_force.y=2
    # pose.acceleration_or_force.z=2

def update_msg(msg):
    msg.header = mavros.setpoint.Header(
                    frame_id=msg.header.frame_id,
                    stamp=rospy.Time.now())

def local_position_cb(topic):
    """local position subscriber callback function
    """
    current_position.is_init = True
    current_position.x = topic.latitude
    current_position.y = topic.longitude
    current_position.z = topic.altitude

def is_reached(setpoint):
    """Check if the UAV reached the destination
    """
    x = setpoint.latitude
    y = setpoint.longitude
    z = setpoint.altitude
    if (abs(current_position.x-x) < precision and
            abs(current_position.y-y) < precision and
            abs(current_position.z-z) < precision):
        print "Point reached!"
        return True
    else:
        return False

def is_overtime(timestamp, overtime):
    if ((rospy.Time.now() - timestamp) > rospy.Duration(overtime)):
        print "Flight attempt over time!"
        return True
    else:
        return False
   

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
    # setup global pub
    setpoint_local_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'global'),mavros_msgs.msg.GlobalPositionTarget ,queue_size=10 )

    # setup setpoint_msg
    setpoint_msg = mavros_msgs.msg.GlobalPositionTarget(
            header=mavros.setpoint.Header(
                frame_id="global_gps",
                stamp=rospy.Time.now()),
            )

    # setup global sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('global_position', 'global'),
    	sensor_msgs.msg.NavSatFix, local_position_cb)

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_GLOBAL_GOTO')

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    raw_setpoint_position.x=float(setpoint_arg[0])
    raw_setpoint_position.y=float(setpoint_arg[1])
    raw_setpoint_position.z=float(setpoint_arg[2])
    over_time = float(setpoint_arg[3])
    print "X: {}, Y: {}, Z: {}".format(raw_setpoint_position.x,
    	raw_setpoint_position.y, raw_setpoint_position.z)

    # setup setpoint poisiton and prepare to publish the position
    # set_target(setpoint_msg,
    # 	setpoint_position.x,
    # 	setpoint_position.y,
    # 	setpoint_position.z)

    pre_flight = 'neutral'
    init_time = rospy.Time.now()
    # In this while loop, do the job.
    # Waiting for receive the first current position
    while(current_position.is_init is False):
        continue
    if (raw_setpoint_position.z - current_position.z >2):
        pre_flight = 'ascending'
        # ascending
        set_target(setpoint_msg,
            current_position.x,
            current_position.y,
            raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            update_msg(setpoint_msg)
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
        # flight to the target
        set_target(setpoint_msg,
            raw_setpoint_position.x,
            raw_setpoint_position.y,
            raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            update_msg(setpoint_msg)
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()

    elif (current_position.z - raw_setpoint_position.z >2):
        pre_flight = 'descending'
        # flight to the target first
        set_target(setpoint_msg,
            raw_setpoint_position.x,
            raw_setpoint_position.y,
            current_position.z)
        while(not is_reached(setpoint_msg)):
            update_msg(setpoint_msg)
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
        # descending
        set_target(setpoint_msg,
            raw_setpoint_position.x,
            raw_setpoint_position.y,
            raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            update_msg(setpoint_msg)
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()


    else:
        set_target(setpoint_msg,
            raw_setpoint_position.x,
            raw_setpoint_position.y,
            raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
    
    # TODO: publish the task status as FINISHING
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

