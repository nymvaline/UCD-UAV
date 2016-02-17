#!/usr/bin/env python
import sys
import platform
if (platform.uname()[1]=='ubuntu'):
	sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname[1]=='edison'):
	sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

import rospy
# import ROS libraries
import rospy
import mavros
from mavros.utils import *
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
from datetime import datetime
from UAV_Task import *
# import utilities
import thread
import math
import sys
import signal
import subprocess


rospy.loginfo("HAHAHA")
print "Hahaha Im in the sky!"

