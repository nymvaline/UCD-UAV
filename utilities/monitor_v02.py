#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros_msgs.msg
import geometry_msgs.msg 
import sensor_msgs.msg
import time
from datetime import datetime

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
		self.current_pose_global = _vector3()
		self.setpoint_pose_global = _vector3()
		self.vel_linear = _vector3()
		self.vel_angular = _vector3()
		self.mode = "None"
		self.arm = "None"
		self.guided = "None"
		self.timestamp = float(datetime.utcnow().strftime('%S.%f'))
		self.conn_delay = 0.0
		rospy.init_node('UAV_Monitor')
		mavros.set_namespace("/mavros")
		# setup local_position sub
		self.local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
			SP.PoseStamped, self._local_position_callback)
		self.setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_position', 'local'),
			SP.PoseStamped, self._setpoint_position_callback)
		self.position_global_sub = rospy.Subscriber(mavros.get_topic('global_position', 'global'),
			sensor_msgs.msg.NavSatFix, self._global_position_callback)
		self.setpoint_global_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'global'),
			mavros_msgs.msg.GlobalPositionTarget, self._globa_setpoint_position_callback)
		self.velocity_sub = rospy.Subscriber(mavros.get_topic('local_position','velocity'),
			geometry_msgs.msg.TwistStamped, self._velocity_callback)
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

	def _global_position_callback(self, topic):
		self.current_pose_global.x = topic.latitude
		self.current_pose_global.y = topic.longitude
		self.current_pose_global.z = topic.altitude


	def _globa_setpoint_position_callback(self, topic):
		self.setpoint_pose_global.x = topic.latitude
		self.setpoint_pose_global.y = topic.longitude
		self.setpoint_pose_global.z = topic.altitude

	def _velocity_callback(self, topic):
		self.vel_linear.x = topic.twist.linear.x
		self.vel_linear.y = topic.twist.linear.y
		self.vel_linear.z = topic.twist.linear.z
		self.vel_angular.x = topic.twist.angular.x
		self.vel_angular.y = topic.twist.angular.y
		self.vel_angular.z = topic.twist.angular.z


	def _state_callback(self, topic):
		self._calc_delay()
		self.mode = topic.mode   
		self.guided = topic.guided
		self.arm = topic.armed

	def _calc_delay(self):
		tmp = float(datetime.utcnow().strftime('%S.%f'))
		if tmp<self.timestamp:
			# over a minute
			self.conn_delay = 60.0 - self.timestamp + tmp
		else:
			self.conn_delay = tmp - self.timestamp
		self.timestamp = tmp

	def get_mode(self):
		return self.mode

	def get_arm(self):
		return self.arm

	def get_current_pose(self):
		return self.current_pose

	def get_setpoint_pose(self):
		return self.setpoint_pose

	def get_current_pose_global(self):
		return self.current_pose_global

	def get_setpoint_pose_global(self):
		return self.setpoint_pose_global

	def get_velocity(self):
		return (self.vel_linear, self.vel_angular)

	def get_guided(self):
		return self.guided

	def get_delay(self):
		return self.conn_delay
	



UAV = UAV_class()



def _UAV_status(screen):
	# mode
	screen.addstr(1,2+0*H_SPACE, "UAV_MODE: " + str(UAV.get_mode())+'    ')
	# arm
	screen.addstr(1,2+1*H_SPACE, "UAV_ARM: " + str(UAV.get_arm())+'    ')
	# Guided
	screen.addstr(1,3+2*H_SPACE, "UAV_GUIDED: " + str(UAV.get_guided())+'    ')

	size = screen.getmaxyx()
	screen.hline(3, 2, '-', size[1]-4)

	pass

def _UAV_current_pose(screen):
	screen.addstr(4, 2+0*H_SPACE, "Current Position:")
	temp = UAV.get_current_pose()
	screen.addstr(5, 2+0*H_SPACE, "X: "+str(format(temp.x, '.4f')))
	screen.addstr(6, 2+0*H_SPACE, "y: "+str(format(temp.y, '.4f')))
	screen.addstr(7, 2+0*H_SPACE, "z: "+str(format(temp.z, '.4f')))
	pass

def _UAV_setpoint_pose(screen):
	screen.addstr(4, 2+1*H_SPACE, "Setpoint Position:")
	temp = UAV.get_setpoint_pose()
	screen.addstr(5, 2+1*H_SPACE, "X: "+str(format(temp.x, '.4f')))
	screen.addstr(6, 2+1*H_SPACE, "y: "+str(format(temp.y, '.4f')))
	screen.addstr(7, 2+1*H_SPACE, "z: "+str(format(temp.z, '.4f')))
	pass

def _UAV_current_pose_global(screen):
	screen.addstr(4, 2+2*H_SPACE, "GPS Position:")
	temp = UAV.get_current_pose_global()
	screen.addstr(5, 2+2*H_SPACE, "X: "+str(format(temp.x, '.4f')))
	screen.addstr(6, 2+2*H_SPACE, "y: "+str(format(temp.y, '.4f')))
	screen.addstr(7, 2+2*H_SPACE, "z: "+str(format(temp.z, '.4f')))
	pass

def _UAV_setpoint_pose_global(screen):
	screen.addstr(4, 2+3*H_SPACE, "GPS Setpoint:")
	temp = UAV.get_setpoint_pose_global()
	screen.addstr(5, 2+3*H_SPACE, "X: "+str(format(temp.x, '.4f')))
	screen.addstr(6, 2+3*H_SPACE, "y: "+str(format(temp.y, '.4f')))
	screen.addstr(7, 2+3*H_SPACE, "z: "+str(format(temp.z, '.4f')))
	pass

def _UAV_velocity(screen):
	screen.addstr(9, 2+0*H_SPACE, "Linear velocity:")
	screen.addstr(9, 2+1*H_SPACE, "Angular velocity:")
	(temp_linear, temp_angular) = UAV.get_velocity()
	screen.addstr(10, 2+0*H_SPACE, "Lin X: "+str(format(temp_linear.x, '.4f')))
	screen.addstr(11, 2+0*H_SPACE, "Lin y: "+str(format(temp_linear.y, '.4f')))
	screen.addstr(12, 2+0*H_SPACE, "Lin z: "+str(format(temp_linear.z, '.4f')))

	screen.addstr(10, 2+1*H_SPACE, "Ang X: "+str(format(temp_angular.x, '.4f')))
	screen.addstr(11, 2+1*H_SPACE, "Ang y: "+str(format(temp_angular.y, '.4f')))
	screen.addstr(12, 2+1*H_SPACE, "Ang z: "+str(format(temp_angular.z, '.4f')))

	pass

def _UAV_delay(screen):
	screen.addstr(2, 2+0*H_SPACE, "Connection delay: "+str(format(UAV.get_delay(), '.4f'))+" s")


def main(argc):
	# initialize
	screen = curses.initscr()
	key = 0

	screen.nodelay(1)
	screen.clear()
	screen.border(0)


	while(key != ord('q')):
		screen.refresh()
		_UAV_status(screen)
		_UAV_current_pose(screen)
		_UAV_setpoint_pose(screen)
		_UAV_current_pose_global(screen)
		_UAV_setpoint_pose_global(screen)
		_UAV_velocity(screen)
		_UAV_delay(screen)

		time.sleep(0.1)
		key = screen.getch()

if __name__ == '__main__':
    try:
	curses.wrapper(main)
    finally:
    	curses.endwin()
        # pass


