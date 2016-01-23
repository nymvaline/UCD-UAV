#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
import mavros.utils;
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
import serial
import abc
import datetime

# DART Functions
import ipcs
import state
import pos


class TaskManager(object):
    def __init__(self, other=None):
        self.tasks = []

    def add(self, task):
        self.tasks.append(task)

    def run(self):
        for i in range(0, len(self.tasks)):
            print("[INFO] Executing task %d" % i)
            self.tasks[i].execute()


class Task(object):
    def __init__(self):
        self.rate = rospy.Rate(10)

    def execute(self):
        while (not self.__done__()):
            self.keepalive()
            self.__execute__()
            self.rate.sleep()

    def keepalive(self):
        setpoint_msg = mavros.setpoint.PoseStamped(
                header=mavros.setpoint.Header(
                        frame_id="att_pose",
                        stamp=rospy.Time.now()),
        )
        setpoint_msg.pose.position.x = pos.setpoint.x
        setpoint_msg.pose.position.y = pos.setpoint.y
        setpoint_msg.pose.position.z = pos.setpoint.z
        ipcs.SETPOINT_LOCAL_PUB.publish(setpoint_msg)

    @abc.abstractmethod
    def __execute__(self):
        return True

    @abc.abstractmethod
    def __done__(self):
        return False


class InitTask(Task):
    def __init__(self, setpoint):
        Task.__init__(self)
        self.count = 0
        pos.setpoint.copy(setpoint)

    def __execute__(self):
        pass

    def __done__(self):
        self.count += 1
        if (self.count > 50 and state.UAV_state.connected):  # wait for FCU connection
            print("[INFO] \tTransmitted Initial Set-points (%f,%f,%f)." % (pos.setpoint.x, pos.setpoint.y, pos.setpoint.z))
            return True
        else:
            return False


class Arm(Task):
    def __init__(self):
        Task.__init__(self)

    def __execute__(self):
        mavros.command.arming(True)

    def __done__(self):
        if (state.UAV_state.armed):
            print '[INFO] \tVehicle Armed.'
            return True
        else:
            return False

class Disarm(Task):
    def __init__(self):
        Task.__init__(self)

    def __execute__(self):
        mavros.command.arming(False)

    def __done__(self):
        if (state.UAV_state.armed):
            print '[INFO] \tVehicle Dis-Armed.'
            return True
        else:
            return False


class ChangeMode(Task):
    def __init__(self, mode):
        Task.__init__(self)
        self.mode = mode

    def __execute__(self):
        state.set_mode(0, self.mode)

    def __done__(self):
        if (state.UAV_state.mode == self.mode):
            print("[INFO] \tVehicle Mode Changed to '%s'." % self.mode)
            return True
        else:
            return False


class GoToLocal(Task):
    def __init__(self, setpoint, precision=0.5):
        Task.__init__(self)
        self.precision = precision
        self.setpoint = pos.vector3(setpoint.x, setpoint.y, setpoint.z)

    def __execute__(self):
        print("[INFO] \tTravelling to (%f,%f,%f)." % (self.setpoint.x, self.setpoint.y, self.setpoint.z))
        print("[INFO] \tCurrent (%f,%f,%f)." % (pos.curpoint.x, pos.curpoint.y, pos.curpoint.z))
        pos.setpoint.copy(self.setpoint)

    def __done__(self):
        if (abs(pos.curpoint.x - self.setpoint.x) < self.precision and
                    abs(pos.curpoint.y - self.setpoint.y) < self.precision and
                    abs(pos.curpoint.z - self.setpoint.z) < self.precision):
            print("[INFO] \t(%f,%f,%f) reached." % (self.setpoint.x, self.setpoint.y, self.setpoint.z));
            return True
        else:
            return False


class Stay(Task):
    def __init__(self, duration=0.):
        Task.__init__(self)
        self.cd = False
        self.duration = duration
        self.start = -1.0

    def __execute__(self):
        if(not self.cd):
            self.cd = True
            self.start = rospy.Time.now()
            print("[INFO] \tStaying for %f seconds." % self.duration)

    def __done__(self):
        if ( self.cd and (rospy.Time.now() - self.start) > rospy.Duration(self.duration, 0)):
            print("[INFO] \tStayed for %f seconds." % self.duration)
            return True
        else:
            return False


class GoToGPS(Task):
    '''
    This is task that guide the UAV to GPS position
    '''
    pass


class GetData(Task):
    def __init__(self):
        pass
