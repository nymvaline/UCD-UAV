#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
'''
This is the TASK_LOCAL_GOTO
This task will guide the drone to the destination point 
by keeping sending setpoint_local
This task is able to check is_reached
once the UAV reached the point, the task will exit
TODO:
implement an appropriate platfrom recognition
implement task publisher that talking to main TCS process
that this TASK has finished its job.
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
import time
from datetime import datetime

import TCS_util

# Declare some global variables.
current_position = TCS_util.vector3()
setpoint_msg = 0
setpoint_position = TCS_util.vector3()
raw_setpoint_position = TCS_util.vector3()
# precision setup. normally set it to 0.5m
precision = 0.5
# setup frame_id
frame_id='LOCAL_GOTO'

def set_target(pose, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of 
    mavros.setpoint.PoseStamped
    """
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.header=mavros.setpoint.Header(
        frame_id="local_pose",
        stamp=rospy.Time.now())
    #pose.coordinate_frame = 0
    #pose.type_mask = 8+16+32+64+128+256+1024+2048

def update_msg(msg):
    msg.header = mavros.setpoint.Header(
                    frame_id=msg.header.frame_id,
                    stamp=rospy.Time.now())

def local_position_cb(topic):
    """local position subscriber callback function
    """
    current_position.is_init = True
    current_position.x = topic.pose.position.x
    current_position.y = topic.pose.position.y
    current_position.z = topic.pose.position.z

def is_reached(setpoint):
    """Check if the UAV reached the destination
    """
    x = setpoint.position.x
    y = setpoint.position.y
    z = setpoint.position.z
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

def fly_to_setpoint(raw_setpoint_position, task_watchdog, over_time, rate):
    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)
    #rospy.Publisher(mavros.get_topic('setpoint_raw', 'local'),mavros_msgs.msg.PositionTarget ,queue_size=10 )

    setpoint_msg = mavros_msgs.msg.PositionTarget(
            header=mavros.setpoint.Header(
                frame_id="local_pose",
                stamp=rospy.Time.now()),
            )


    # setup local sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    	SP.PoseStamped, local_position_cb)

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

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(25) # Increased from 20, maybe failsafe mode is not?
    mavros.set_namespace('/mavros')

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_LOCAL_GOTO')

    # interpret the input position
    setpoint_arg = sys.argv[1].split(' ')
    raw_setpoint_position.x=float(setpoint_arg[0])
    raw_setpoint_position.y=float(setpoint_arg[1])
    raw_setpoint_position.z=float(setpoint_arg[2])
    over_time = float(setpoint_arg[3])
    print "X: {}, Y: {}, Z: {}".format(raw_setpoint_position.x,
    	raw_setpoint_position.y, raw_setpoint_position.z)

    fly_to_setpoint(raw_setpoint_position, task_watchdog, over_time, rate)

    # TODO: publish the task status as FINISHING
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

