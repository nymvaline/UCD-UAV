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
from mavros_msgs.msg import CommandCode
from mavros_msgs.srv import SetMode
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
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')


    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),
        mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    # local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    #     SP.PoseStamped, _local_position_callback)
    # /mavros/setpoint_raw/target_local
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
        mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    # setup publisher
    # /mavros/setpoint/position/local
    setpoint_local_pub =  mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
    # /mavros/cmd/takeoff
    cmd_takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
    # /mavros/cmd/command
    cmd_long = rospy.ServiceProxy('/mavros/cmd/command', mavros_msgs.srv.CommandLong) 
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  
    # /mavros/mission/pull
    waypoint_pull = rospy.ServiceProxy('/mavros/mission/pull', mavros_msgs.srv.WaypointPull) 
    # /mavros/mission/push
    waypoint_push = rospy.ServiceProxy('/mavros/mission/push', mavros_msgs.srv.WaypointPush) 
    # /mavros/mission/clear
    waypoint_clear = rospy.ServiceProxy('/mavros/mission/clear', mavros_msgs.srv.WaypointClear) 


    setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )

    #read task list
    Task_mgr = TCS_util.Task_manager('task_list.log')

    # start setpoint_update instance
    setpoint_keeper = TCS_util.update_setpoint(rospy)

    # wait for FCU connection
    while(not UAV_state.connected):
        rate.sleep()

    #set manual
    # set_mode(SetMode.MAV_MODE_MANUAL_DISARMED,'')

    #set home
    result=cmd_long(broadcast=True, command=CommandCode.CMD_DO_SET_HOME, confirmation=0, param1=1)
    if(result.success):
        print("Home as been set up")


    #set waypoint
    waypoint_clear()
    new_waypoint_list = []
    new_waypoint = mavros_msgs.msg.Waypoint(frame=3, is_current=True, command=22,autocontinue=True, param1=5, param4=0, x_lat=47.3978800,y_long=8.5455920,z_alt=10)
    new_waypoint_list.append(new_waypoint)
    new_waypoint = mavros_msgs.msg.Waypoint(frame=3, command=16,autocontinue=True, param1=5, param2=0.5,param3=0, param4=0, x_lat=47.3978800,y_long=8.5456,z_alt=10)
    new_waypoint_list.append(new_waypoint)
    new_waypoint = mavros_msgs.msg.Waypoint(frame=3, command=21,autocontinue=True, param1=0, param4=0, x_lat=47.3978800,y_long=8.5457,z_alt=0)
    new_waypoint_list.append(new_waypoint)
    result = waypoint_push(new_waypoint_list)
    # result = waypoint_pull()
    # while(result.wp_received!=len(new_waypoint_list)):
    #     result = waypoint_pull()
    # print("There are {} waypoints".format(result.wp_received))


    # initialize the setpoint
    setpoint_msg.pose.position.x = 0
    setpoint_msg.pose.position.y = 0
    setpoint_msg.pose.position.z = 3
    	


    # send 100 setpoints before starting
    # for i in range(0,50):
    #     setpoint_local_pub.publish(setpoint_msg)
    #     rate.sleep()

    # result = set_mode(0,'OFFBOARD')
    # cmd_takeoff(min_pitch=15, yaw=0, latitude=47.3978800, longitude=8.5455920, altitude=10)
    for i in range(0,10):
        set_mode(0,'AUTO.TAKEOFF')
        rate.sleep()
    mavros.command.arming(True)
    for i in range(0,10):
        set_mode(0,'AUTO.MISSION')
        rate.sleep()
    # cmd_long(broadcast=True, command=CommandCode.CMD_MISSION_START, confirmation=0,param1=1, param2=3)
    while(True):
        # cmd_long(broadcast=True, command=92, confirmation=0,param1=1)
        # cmd_long(broadcast=True, command=22, confirmation=0,param1=5, param2=1, param3=1, param4=0, param5=47.3978800,param6=8.5455920,param7=10)

        if(UAV_state.armed!=True):
            return
        rate.sleep()
    last_request = rospy.Time.now()


    # enter the main loop
    while(True):
        # print "Entered whiled loop"
        if( UAV_state.mode != "OFFBOARD" and
            (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if( set_mode(0,'OFFBOARD').success):
                print "Offboard enabled"
            last_request = rospy.Time.now()
        else:
            if(not UAV_state.armed and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if( mavros.command.arming(True)):
                    print "Vehicle armed"
                last_request = rospy.Time.now()

        # update setpoint to stay in offboard mode
        setpoint_keeper.update()
        

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