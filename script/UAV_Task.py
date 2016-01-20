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

class vector3(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class Task_GOTO_Local(object):
    def __init__(self, setpoint_publish):
        self.setpoint_pub = setpoint_publish
        self.current_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
            SP.PoseStamped, self._local_position_callback)
        self.current = vector3()
        self.msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )
        

        self.x = 0
        self.y = 0
        self.z = 0
        # default precision is 0.5
        self.precision = 0.5    

    def goto(self, x=0, y=0, z=0, xyz=None):
        if (xyz==None):
            self.x = x
            self.y = y
            self.z = z
        else:
            self.x = xyz.x
            self.y = xyz.y
            self.z = xyz.z
        self._set_pose(self.msg, x, y, z)
        self.setpoint_pub.publish(self.msg)

    def check_task(self):
        if self.is_reached(self.current, self.msg):
            return True
        else:
            return False

    def is_reached(self, current, setpoint):
        if (abs(current.x-setpoint.pose.position.x) < self.precision and
            abs(current.y-setpoint.pose.position.y) < self.precision and
            abs(current.z-setpoint.pose.position.z) < self.precision):
            print "Point reached!"
            return True
        else:
            return False

    def _set_pose(self, pose, x, y, z):
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.header=mavros.setpoint.Header(
                    frame_id="att_pose",
                    stamp=rospy.Time.now())

    def _local_position_callback(self,topic):
        self.current.x = topic.pose.position.x
        self.current.y = topic.pose.position.y
        self.current.z = topic.pose.position.z

    def set_precision(self, value):
        self.precision = value

    def get_setpoint_pub(self):
        return self.setpoint_pub

    def get_current(self):
        return self.current


class Task_Stay(Task_GOTO_Local):
    def __init__(self, setpoint_publish):
        super(Task_Stay, self).__init__(setpoint_publish)
        self.timer = rospy.Time.now()
        self.stay_pose = vector3();
        self.timer_ready = False
        self.duration = 0

    def stay_at_time(self, duration):
        # if timer is not being reset, reset the timer
        if (self.timer_ready==False):
            print "First enter stay stay_at_time"
            self.timer = rospy.Time.now()
            self.timer_ready = True
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z
            

        self.goto(self.stay_pose.x,
            self.stay_pose.y,
            self.stay_pose.z)
        self.duration = duration

    def stay(self):
        self.goto(xyz=self.get_current())

    def reset_timer(self):
        self.timer_ready = False

    def check_task(self):
        if (rospy.Time.now() - self.timer > rospy.Duration(self.duration)):
            return True
        else:
            return False

    def print_current(self):
        print "x = {} y = {} z = {}".format(self.current.x, self.current.y, self.current.z)

