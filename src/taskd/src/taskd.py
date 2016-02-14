#!/usr/bin/env python

"""
File: taskd
Description: daemon running UAV tasks in the background.

Waits in the background to execute tasks. Persists and record task queue pointer through local file system. Tasks will
be put into a priority queue and execute sequentially.

Queue will be configured by default with json file. Optionally, user may execute client script 'task.py' to add tasks
during daemon runtime.

By design, taskd publishes to the following ROS topics:
    setpoint_raw: publishes position/velocity setpoints to latch the UAV to destination.

By design, taskd publshes to the following UCD topics:
    /task/status: returns status of the current task.

Version: v1
Author: Jun D. Ouyang (jdouyang@ucdavis.edu)
"""

# ROS libraries
import rospy
import mavros

# UCD libraries
import objects

def main():
    pass



if __name__ == '__main__':
    main()
