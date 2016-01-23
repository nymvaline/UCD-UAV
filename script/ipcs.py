#!/usr/bin/env python

import rospy
import mavros
import mavros.utils
#from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv

# DART Functions
import tasks
import state
import pos

def subscribe():
    global STATE_SUB
    global SETPOINT_LOCAL_SUB
    global SETPOINT_LOCAL_PUB
    #subscribers
    STATE_SUB = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, state._state_callback)
    SETPOINT_LOCAL_SUB = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),mavros_msgs.msg.PositionTarget, pos._current_position_callback)

    #publishers
    SETPOINT_LOCAL_PUB = mavros.setpoint.get_pub_position_local(queue_size=20)
