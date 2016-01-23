#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
import datetime

# import utilities
import math
import sys
import signal

# DART Functions
import ipcs
import tasks
import state
import pos

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)


def main():
    rospy.init_node('default_offboard', anonymous=True)
    mavros.set_namespace('/mavros')

    ipcs.subscribe()
    state.state_setup()

    # create task instance
    taskMgr = tasks.TaskManager()
    taskMgr.add(tasks.InitTask(pos.setXYZ(0,0,0)))
    taskMgr.add(tasks.Arm())
    taskMgr.add(tasks.ChangeMode("OFFBOARD"))
    taskMgr.add(tasks.GoToLocal(pos.setZ(5)))
    taskMgr.add(tasks.Stay(10.0))
    taskMgr.add(tasks.GoToLocal(pos.setXYZ(5,5,10)))
    taskMgr.add(tasks.Stay(10.0))
    taskMgr.add(tasks.GoToLocal(pos.setXYZ(0,5,4)))
    taskMgr.add(tasks.Stay(10.0))
    taskMgr.add(tasks.GoToLocal(pos.setXYZ(0,0,4)))
    taskMgr.add(tasks.Stay(10.0))
    taskMgr.add(tasks.ChangeMode("AUTO.LAND"))
    taskMgr.add(tasks.Disarm())
    taskMgr.run()

    return 0




if __name__ == '__main__':
    main()
