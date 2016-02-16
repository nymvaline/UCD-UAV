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
import pyserial
from datetime import datetime

# import mraa
import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')
import mraa


class vector3(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class update_setpoint(object):
    def __init__(self):
        self.update_flag='LOCAL'

        #setup local position 
        self.local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)
        self.local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
        mavros_msgs.msg.PositionTarget, self._local_cb)
        self.local_last_pos=vector3()
        self.local_msg=mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )

        #setup GPS position
        self.GPS_pub =  mavros.setpoint.get_pub_position_global(queue_size=10)
        self.GPS_last_pos=vector3()
        self.GPS_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_global'),
        mavros_msgs.msg.PositionTarget, self._GPS_cb)
        self.GPS_msg=mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )

    def _local_cb(self, topic):
        self.update_flag='LOCAL'
        self.local_last_pos.x=topic.pose.position.x;
        self.local_last_pos.y=topic.pose.position.y;
        self.local_last_pos.z=topic.pose.position.z;

    def _GPS_cb(self, topic):
        self.update_flag='GPS'
        self.GPS_last_pos.x=topic.pose.position.x;
        self.GPS_last_pos.y=topic.pose.position.y;
        self.GPS_last_pos.z=topic.pose.position.z;

    def _set_pose(self, pose, pos):
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z
        pose.header=mavros.setpoint.Header(
                    frame_id="att_pose",
                    stamp=rospy.Time.now())

    def update(self):
        if (update_flag=='LOCAL'):
            self._set_pose(self.local_msg, self.local_last_pos)
            self.local_pub.publish(self.local_msg)
        if (update_flag=='GPS'):
            self._set_pose(self.GPS_msg, self.GPS_last_pos)
            self.GPS_pub.publish(self.GPS_msg)
        pass

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
        if (self.init_ready==False):
            print "First enter stay stay_at_time"
            self.timer = rospy.Time.now()
            self.init_ready = True
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z
            

        self.goto(self.stay_pose.x,
            self.stay_pose.y,
            self.stay_pose.z)
        self.duration = duration

    def stay(self):
        if(self.init_ready==False):
            self.init_ready = True
            # need hard copy here because simple self.stay_pose = self.current will give reference to 
            # self.stay_pose and then self.stay_pose will change following the self.current
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z

        self.goto(self.stay_pose.x,
            self.stay_pose.y,
            self.stay_pose.z)

    def reset_stay(self):
        # need to reset say flag after use
        self.init_ready = False

    def check_task(self):
        if (rospy.Time.now() - self.timer > rospy.Duration(self.duration)):
            return True
        else:
            return False

    def print_current(self):
        print "x = {} y = {} z = {}".format(self.current.x, self.current.y, self.current.z)


class Task_GOTO_GPS(object):
    '''
    This is task that guide the UAV to GPS position
    '''
    pass
