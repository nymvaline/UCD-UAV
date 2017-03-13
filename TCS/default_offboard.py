#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg
import sensor_msgs.srv
import time
from datetime import datetime
from UAV_Task import *
import TCS_util
# import utilities
import thread
import math
import sys
import signal
import subprocess



def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


current_pose = vector3()
UAV_state = mavros_msgs.msg.State()



def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided



def _setpoint_position_callback(topic):
    pass

found_home = False
home_pos = sensor_msgs.msg.NavSatFix()

def _home_pos_callback(topic):
    global found_home # TODO (jasmine): do this without global variables
    if not found_home:
        home_pos.latitude = topic.latitude
        home_pos.longitude = topic.longitude
        home_pos.altitude = topic.altitude
        print "Found home!"
        found_home = True

def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now())



def is_reached(current, setpoint):
    if (abs(current.x-setpoint.pose.position.x) < 0.5 and
        abs(current.y-setpoint.pose.position.y) < 0.5 and
        abs(current.z-setpoint.pose.position.z) < 0.5):
        print "Point reached!"
        return True
    else:
        return False

def update_setpoint():
    pass


def main():
    print "Starting TCS"
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(25) # increased from 20, maybe failsafe mode is not?
    mavros.set_namespace('/mavros')


    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),
        mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
        mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    # Initial global position publisher. Assuming there isn't already one?
    position_local_sub = rospy.Subscriber(mavros.get_topic('global_position', 'raw', 'fix'),
    	sensor_msgs.msg.NavSatFix, _home_pos_callback)
    startpoint_global_pub = rospy.Publisher('home_point', sensor_msgs.msg.NavSatFix, queue_size=10, latch=True) # Latch it because we only really need it once
    print "Waiting for home!"
    while(not found_home):
        rate.sleep()
    startpoint_global_pub.publish(home_pos);
    print "Sent startpoint! {} {} {}".format(home_pos.latitude, home_pos.longitude, home_pos.altitude)

    # setup publisher
    # /mavros/setpoint/position/local
    setpoint_local_pub =  mavros.setpoint.get_pub_position_local(queue_size=10)
    #setpoint_global_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'global'),
    #    mavros_msgs.msg.GlobalPositionTarget, queue_size=10)
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

    setpoint_global_msg = mavros_msgs.msg.GlobalPositionTarget(
            header=mavros.setpoint.Header(
                frame_id="global_pose",
                stamp=rospy.Time.now()),
            )


    #read task list

    Task_mgr = TCS_util.Task_manager('task_list.log')

    # start setpoint_update instance
    setpoint_keeper = TCS_util.update_setpoint(rospy)

    # wait for FCU connection
    while(not UAV_state.connected):
        rate.sleep()

    # initialize the setpoint
    setpoint_msg.pose.position.x = 0
    setpoint_msg.pose.position.y = 0
    setpoint_msg.pose.position.z = 3

    setpoint_global_msg.coordinate_frame = 11
    # Ignore velocity x y z, acceleration x y z, and yaw
    setpoint_global_msg.type_mask = 8+16+32+128+256#8+16+32+64+128+256+1024+2048
    setpoint_global_msg.latitude = 47.3978800
    setpoint_global_msg.longitude = 8.5455920
    setpoint_global_msg.altitude = 10
    setpoint_global_msg.yaw = 90
    setpoint_global_msg.yaw_rate =10

    #mavros.command.arming(True)

    # send 100 setpoints before starting
    for i in range(0,100):
        setpoint_local_pub.publish(setpoint_msg)
        #setpoint_global_pub.publish(setpoint_msg)
        rate.sleep()

    #set_mode(0,'OFFBOARD')
    print("Pre start finished!")

    last_request = rospy.Time.now()


    # enter the main loop
    while(True):
        # print "Entered while loop"
        # Spin while we're saiting for offboard instead of making it try to go to offboard mode
        if( UAV_state.mode != "OFFBOARD" ): # and
            if (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                #print "Not in OFFBOARD"
                last_request = rospy.Time.now()
            #if( set_mode(0,'OFFBOARD').success):
            #    print "Offboard enabled"
            # Send setpoints so that we can go in.
            #setpoint_local_pub.publish(setpoint_msg)
            setpoint_keeper.update()
            setpoint_local_pub.publish(setpoint_msg)
            #setpoint_global_pub.publish(setpoint_msg)
            continue
        else:
            if(not UAV_state.armed): # and
                if (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                    last_request = rospy.Time.now()
                    print "Not armed"
                # Spin while we're waiting to manually arm
                #if( mavros.command.arming(True)):
                #    print "Vehicle armed"
                setpoint_local_pub.publish(setpoint_msg)
                #setpoint_global_pub.publish(setpoint_msg)
                setpoint_keeper.update() # TODO (jasmine) check that this isn't redundant
                continue

        # update setpoint to stay in offboard mode
        #setpoint_local_pub.publish(setpoint_msg)
        setpoint_keeper.update() # TODO (jasmine) check that this isn't redundant # this isn't running as I expect right now
        

        if(Task_mgr.task_finished()):
            # If the current task has been done
            rospy.loginfo("Current task is finished!")
            if (not Task_mgr.alldone()):
                # If there are tasks left
                Task_mgr.nexttask()

            else:
                # Current task has been done and no task left
                rospy.loginfo("All tasks have been done!")
                while (UAV_state.mode != "AUTO.LAND"):
                    set_mode(0,'AUTO.LAND')
                    rate.sleep()
                return 0


        rate.sleep()
    return 0




if __name__ == '__main__':
    main()
