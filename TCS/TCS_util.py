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
import geometry_msgs
import time
import serial
from datetime import datetime

# import mraa
import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')



class vector3(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class update_setpoint(object):
    def __init__(self,rospy):
        self.update_flag='LOCAL'
        self.frame_id='UPDATE_SETPOINT'
        self.timestamp=rospy.Time.now()

        #setup local position 
        self.local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)
        self.local_sub = rospy.Subscriber(mavros.get_topic('setpoint_position', 'local'),
        geometry_msgs.msg.PoseStamped, self._local_cb)
        self.local_last_pos=vector3()
        self.local_msg=mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id=self.frame_id,
                stamp=rospy.Time.now()),
            )

        #setup GPS position
        # self.GPS_pub =  rospy.Publisher(mavros.get_topic('setpoint_raw', 'global'),
        #  mavros_msgs.msg.GlobalPositionTarget, queue_size=10)
        # self.GPS_last_pos=0
        # self.GPS_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_global'),
        # mavros_msgs.msg.GlobalPositionTarget, self._GPS_cb)
        # self.GPS_msg=0

    def _local_cb(self, topic):
        if (topic.header.frame_id == self.frame_id):
            # ignore the msgs sent by myselfs
            return

        self.update_flag='LOCAL'
        self.timestamp=rospy.Time.now()
        # rospy.loginfo("setpoint_raw target_local get: x=%s, y=%s, z=%s,", 
            topic.pose.position.x, topic.pose.position.y, topic.pose.position.z)
        self.local_last_pos.x=topic.pose.position.x;
        self.local_last_pos.y=topic.pose.position.y;
        self.local_last_pos.z=topic.pose.position.z;

    # def _GPS_cb(self, topic):
    #     self.update_flag='GPS'
    #     self.GPS_last_pos = topic

    def _set_pose(self, pose, pos):
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z
        pose.header=mavros.setpoint.Header(
                    frame_id=self.frame_id,
                    stamp=rospy.Time.now())

    def update(self):
        if(rospy.Time.now()-self.timestamp < (rospy.Duration(0.05))):
            # if setpoint was publishing on time, dont bother to send again
            return
        # rospy.loginfo("Setpoint_keeper sending the setpoint!")
        if (self.update_flag=='LOCAL'):
            self._set_pose(self.local_msg, self.local_last_pos)
            self.local_pub.publish(self.local_msg)
        # if (update_flag=='GPS'):
        #     self._set_pose(self.GPS_msg, self.GPS_last_pos)
        #     self.GPS_pub.publish(self.GPS_msg)
        pass

