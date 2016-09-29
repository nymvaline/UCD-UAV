#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
'''
This is the draft TASK_SIMPLE_DATA
This task will collect data from remote data sensor
'''

# import utilities
import math
import sys
import signal
import subprocess
import os
import platform
if (platform.uname()[1]=='ubuntu'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='edison'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='xiaoguang'):
    # this is workstation in 079 Lab
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
from datetime import datetime
import mraa as m

import TCS_util

# setup mraa
dev = m.Spi(1)
dev.frequency(100000)

job_counter = 0
target_id = 0

def transferAndWait(c):
    r = dev.writeByte(c)
    time.sleep(0.000005)
    # print "Now I read {}".format(c)
    return r

# setup frame_id
frame_id='SIMPLE_DATA'

def is_finished():
    """Check if all the four data been collected
    """
    if (job_counter>=4)
        print "Data collected!"
        return True
    else:
        return False

def collect_data(id):

    loop_counter = 0
    if (loop_counter == 0):
        txbuf=bytearray("STATUS\0".encode('ascii'))
        dev.writeByte(0x7f)
        dev.write(txbuf)
        dev.writeByte(0x17)
        loop_counter+=1
    elif (loop_counter == 1):
        txbuf=bytearray("TEMPERATURE\0".encode('ascii'))
        dev.writeByte(0x7f)
        dev.write(txbuf)
        dev.writeByte(0x17)
        loop_counter+=1
    elif (loop_counter == 2):
        txbuf=bytearray("HUMIDITY\0".encode('ascii'))
        dev.writeByte(0x7f)
        dev.write(txbuf)
        dev.writeByte(0x17)
        loop_counter+=1
    elif (loop_counter == 3):
        txbuf=bytearray("SOIL\0".encode('ascii'))
        dev.writeByte(0x7f)
        dev.write(txbuf)
        dev.writeByte(0x17)
        loop_counter=0

    time.sleep(1)

    # Read 0x7f
    c=transferAndWait(0xff)
    # Read payload Length
    c=transferAndWait(0xff)
    msg_len=int(c)
    if (msg_len<255 and msg_len>1):
        print "msg length is: {}".format(msg_len)
    else:
        print "Wrong msg length"
        time.sleep(2)
        continue

    raw_read=list()
    raw_value=0;
    for i in range(0,msg_len):
        c=transferAndWait(0xff)
        raw_read.append(c)
        # print "0x%x"%int(c)
        # print " "

    # read checksum
    c = transferAndWait(0xff)
    # print "checksum byte is: {}".format(int(c))
    raw_read.append(c)

    # calculate raw_value
    raw_value=(raw_read[0]<<8)&0xff00 | raw_read[1]&0xff

    checksum=raw_read[2]

    if (int((raw_read[0] + raw_read[1]) & 0xff) is not int(checksum&0xff)):
        print "checksum check failed. Drop this message"
        # print "raw_read checksum is {}, checksum is {}".format(((raw_read[0] + raw_read[1]) & 0xff), int(raw_read[2]))
        time.sleep(2)
        continue

    if (loop_counter == 1):
        print "Now STATUS is: %c%c"%(raw_read[0],raw_read[1])
        with open("simplelogger.txt", "a") as myfile:
            myfile.write("{} STATUS is: {}{}".format(rospy.Time.now(),raw_read[0],raw_read[1]))

    if (loop_counter == 2):
        print "Now TEMPERATURE is: {}".format(float(raw_value/10.0))
        with open("simplelogger.txt", "a") as myfile:
            myfile.write("{} TEMPERATURE is: {}".format(rospy.Time.now(),float(raw_value/10.0)))
        
    if (loop_counter == 3):
        print"Now HUMIDITY is: {}".format(float(raw_value/10.0))
        with open("simplelogger.txt", "a") as myfile:
            myfile.write("{} HUMIDITY is: {}".format(rospy.Time.now(),float(raw_value/10.0)))
    
    if (loop_counter == 0):
        print"Now SOIL is: {}".format(float(raw_value/10.0))
        with open("simplelogger.txt", "a") as myfile:
            myfile.write("{} SOIL is: {}".format(rospy.Time.now(),float(raw_value/10.0))


    time.sleep(2) 
   

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_SIMPLE_DATA')

    target_id = int(sys.argv[1])
    print "The target id is: {}".format(target_id)
    # In this while loop, do the job.
    while(not is_finished()):
        

        job_counter+=1
        collect_data(id)
        # TODO: publish the task status as conducting
        task_watchdog.report_running()

        rate.sleep()

    # TODO: publish the task status as FINISHING
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

