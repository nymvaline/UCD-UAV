#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
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
# TODO (jasmine) make this less brittle
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
import math

import TCS_util
import TASK_LOCAL_GOTO

# Declare some global variables.
found_home = False
home_pos = sensor_msgs.msg.NavSatFix()

WGS84_A =  6378137.0
WGS84_B = 6356752.31424518

# TODO make this overlap with local
def is_overtime(timestamp, overtime):
    if ((rospy.Time.now() - timestamp) > rospy.Duration(overtime)):
        print "Flight attempt over time!"
        return True
    else:
        return False

def sin_degrees(input):
    return math.sin(math.radians(input))

def cos_degrees(input):
    return math.cos(math.radians(input))

def local_position_cb(topic):
    """local position subscriber callback function
    """
    current_position.is_init = True
    current_position.x = topic.latitude
    current_position.y = topic.longitude
    current_position.z = topic.altitude

def startpoint_global_cb(topic):
    global found_home # TODO fix me
    found_home = True
    home_pos.latitude = topic.latitude# hopefully there's no issues with objects
    home_pos.longitude = topic.longitude
    home_pos.altitude = topic.altitude # TODO all the other stuff

#see the wikipedia page
def convert_normal_util(latitude):
    return_me = (WGS84_A**2)/math.sqrt(WGS84_A**2*(cos_degrees(latitude))**2 +WGS84_B**2*(sin_degrees(latitude))**2)
    return return_me

def wgs84_to_ecef(nav_sat_fix):
    N = convert_normal_util(nav_sat_fix.latitude)
    # Convert from geodetic to ECEF
    X = (N+nav_sat_fix.altitude)*cos_degrees(nav_sat_fix.latitude)*cos_degrees(nav_sat_fix.longitude) # Todo relative?
    Y = (N+nav_sat_fix.altitude)*cos_degrees(nav_sat_fix.latitude)*sin_degrees(nav_sat_fix.longitude) # Todo relative?
    Z = (((WGS84_B**2)/(WGS84_A**2)) * N + nav_sat_fix.altitude) * sin_degrees(nav_sat_fix.latitude)
    return_me = TCS_util.vector3() # TODO make it wrong?
    return_me.x = X
    return_me.y = Y
    return_me.z = Z
    return return_me

# x =east, y = north, z = up
def ecef_to_local(nav_sat_ref, ecef_ref, ecef):
    # Convert from ECEF to local
    X_diff = ecef.x - ecef_ref.x
    Y_diff = ecef.y - ecef_ref.y
    Z_diff = ecef.z - ecef_ref.z
    return_me = TCS_util.vector3()
    return_me.x = 0 - sin_degrees(nav_sat_ref.longitude) * X_diff + cos_degrees(nav_sat_ref.longitude) * Y_diff
    return_me.y = 0 - sin_degrees(nav_sat_ref.latitude) *cos_degrees(nav_sat_ref.longitude) * X_diff - sin_degrees(nav_sat_ref.latitude) * sin_degrees(nav_sat_ref.longitude) * Y_diff + cos_degrees(nav_sat_ref.latitude) * Z_diff
    return_me.z = cos_degrees(nav_sat_ref.latitude) * cos_degrees(nav_sat_ref.longitude) * X_diff + cos_degrees(nav_sat_ref.latitude)* sin_degrees(nav_sat_ref.longitude) * Y_diff + sin_degrees(nav_sat_ref.latitude)*Z_diff
    return return_me

def wgs84_to_local(nav_sat_fix):
    ecef_ref = wgs84_to_ecef(home_pos)
    ecef_fix = wgs84_to_ecef(nav_sat_fix)
    return ecef_to_local(home_pos, ecef_ref, ecef_fix)

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
    startpoint_global_sub = rospy.Subscriber('home_point', sensor_msgs.msg.NavSatFix, startpoint_global_cb);

    task_watchdog = TCS_util.Task_watchdog('TASK_GLOBAL_GOTO')

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    setpoint_pos = sensor_msgs.msg.NavSatFix()
    setpoint_pos.latitude=float(setpoint_arg[0])
    setpoint_pos.longitude=float(setpoint_arg[1])
    setpoint_pos.altitude=float(setpoint_arg[2])
    over_time = float(setpoint_arg[3])

    init_time = rospy.Time.now()

    while (not found_home):
        task_watchdog.report_running()
        if (is_overtime(init_time, over_time)):
            break

    xyz = wgs84_to_local(setpoint_pos)
    TASK_LOCAL_GOTO.fly_to_setpoint(xyz, task_watchdog, over_time, rate)

    task_watchdog.report_finish()
    return 0

if __name__ == '__main__':
    main()

