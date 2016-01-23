# DART Functions
import ipcs
import tasks
import state


class vector3(object):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def copy(self, other):
        self.x = other.x
        self.y = other.y
        self.z = other.z


setpoint = vector3()
curpoint = vector3()


def setX(x):
    return vector3(x, setpoint.y, setpoint.z)


def setY(y):
    return vector3(setpoint.x, y, setpoint.z)


def setZ(z):
    return vector3(setpoint.x, setpoint.y, z)


def setXYZ(x, y, z):
    return vector3(x, y, z)


def _current_position_callback(topic):
    global curpoint
    curpoint.x = topic.position.x
    curpoint.y = topic.position.y
    curpoint.z = topic.position.z
