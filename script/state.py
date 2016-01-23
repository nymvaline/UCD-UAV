import rospy
import mavros
import mavros.utils
import mavros_msgs.msg

# DART Functions
import ipcs
import tasks
import pos

UAV_state = mavros_msgs.msg.State()

def _state_callback(topic):
    global UAV_state
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def state_setup():
    # setup service
    global set_arming
    global set_mode
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
