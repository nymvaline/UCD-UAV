#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros_msgs.msg
import time


# import utilities
import thread
from math import *
import curses

H_SPACE = 20

class _vector3:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0

class UAV_class:
	def __init__(self):
		self.current_pose = _vector3()
		self.setpoint_pose = _vector3()
		self.mode = "None"
		self.arm = "None"
		rospy.init_node('UAV_Monitor')
		mavros.set_namespace("/mavros")
		# setup local_position sub
		self.local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self._local_position_callback)
		self.setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_position', 'pose'),
                                    SP.PoseStamped, self._setpoint_position_callback)
		self.state_sub = rospy.Subscriber(mavros.get_topic('state'),
					mavros_msgs.msg.State, self._state_callback)
		pass

	def _local_position_callback(self, topic):
		self.current_pose.x = topic.pose.position.x
		self.current_pose.y = topic.pose.position.y
		self.current_pose.z = topic.pose.position.z


	def _setpoint_position_callback(self, topic):
		self.setpoint_pose.x = topic.pose.position.x
		self.setpoint_pose.y = topic.pose.position.y
		self.setpoint_pose.z = topic.pose.position.z

	def _state_callback(self, topic):
		self.mode = topic.mode
		self.guided = topic.guided
		self.arm = topic.armed

	def get_mode(self):
		return self.mode

	def get_arm(self):
		return self.arm

	def get_current_pose(self):
		return self.current_pose

	def get_setpoint_pose(self):
		return self.setpoint_pose

UAV = UAV_class()



def _UAV_status(screen):
	# mode
	screen.addstr(1,2+0*H_SPACE, "UAV_MODE: " + str(UAV.get_mode()))
	# arm
	screen.addstr(1,2+1*H_SPACE, "UAV_ARM: " + str(UAV.get_arm()))

	size = screen.getmaxyx()
	screen.hline(3, 2, '-', size[1]-4)

	pass

def _UAV_current_pose(screen):
	screen.addstr(4, 2+0*H_SPACE, "Current Position:")
	temp = UAV.get_current_pose()
	screen.addstr(5, 2+0*H_SPACE, "X: "+str(temp.x))
	screen.addstr(6, 2+0*H_SPACE, "y: "+str(temp.y))
	screen.addstr(7, 2+0*H_SPACE, "z: "+str(temp.z))
	pass

def _UAV_setpoint_pose(screen):
	screen.addstr(4, 2+1*H_SPACE, "Setpoint Position:")
	temp = UAV.get_setpoint_pose()
	screen.addstr(5, 2+1*H_SPACE, "X: "+str(temp.x))
	screen.addstr(6, 2+1*H_SPACE, "y: "+str(temp.y))
	screen.addstr(7, 2+1*H_SPACE, "z: "+str(temp.z))
	pass



def main(argc):
	# initialize
	screen = curses.initscr()
	key = 0

	screen.nodelay(1)
	screen.clear()
	screen.border(0)


	while(key != ord('q')):
		_UAV_status(screen)
		_UAV_current_pose(screen)
		_UAV_setpoint_pose(screen)
		screen.refresh()
		time.sleep(0.01)
		key = screen.getch()

if __name__ == '__main__':
    try:
	curses.wrapper(main)
    finally:
    	curses.endwin()
        # pass


