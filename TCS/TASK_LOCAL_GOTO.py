#!/usr/bin/env python
# import utilities
import math
import sys
import signal
import subprocess
import os
import platform
if (platform.uname()[1]=='ubuntu'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname[1]=='edison'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from dart.msg import task
import time
from datetime import datetime

import TCS_util


current_position = TCS_util.vector3()
setpoint_msg = 0
setpoint_position = TCS_util.vector3()
precision = 0.5

def set_target(pose, x, y, z):
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.header=mavros.setpoint.Header(
                    frame_id="local_pose",
                    stamp=rospy.Time.now())

def local_position_cb(topic):
    current_position.x = topic.pose.position.x
    current_position.y = topic.pose.position.y
    current_position.z = topic.pose.position.z

def is_reached():
    if (abs(current_position.x-setpoint_position.x) < precision and
            abs(current_position.y-setpoint_position.y) < precision and
            abs(current_position.z-setpoint_position.z) < precision):
        print "Point reached!"
        return True
    else:
        return False

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
    # setup local pub
    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup setpoint_msg
    setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="local_pose",
                stamp=rospy.Time.now()),
            )

    # setup local sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    	SP.PoseStamped, local_position_cb)

    # setup task pub

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    setpoint_position.x=float(setpoint_arg[0])
    setpoint_position.y=float(setpoint_arg[1])
    setpoint_position.z=float(setpoint_arg[2])
    print "X: {}, Y: {}, Z: {}".format(setpoint_position.x,
    	setpoint_position.y, setpoint_position.z)

    set_target(setpoint_msg,
    	setpoint_position.x,
    	setpoint_position.y,
    	setpoint_position.z)
    while(not is_reached()):
    	setpoint_local_pub.publish(setpoint_msg)


    pass


if __name__ == '__main__':
    main()

