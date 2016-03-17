#!/usr/bin/env python
# TX_PIN: J17-7
# RX_PIN: J17-8
# GND PIN: J19-3

import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')

from datetime import datetime
import time
import mraa as m

# Setup and init
dev = m.Spi(1)
print "SPI mode is: {}".format(dev.mode(0))
dev.frequency(100000)
TIME_FORMAT='%m-%d %H:%M:%S'


def transferAndWait(c):
    r = dev.writeByte(c)
    time.sleep(0.000005)
    # print "Now I read {}".format(c)
    return r

loop_counter=0

while(True):
    print "Now counter is: {}".format(loop_counter)
     # send test string
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
        with open('datalog.log', 'a') as outfile:
            outfile.write("%s STATUS: %c%c\n"%(datetime.now().strftime(TIME_FORMAT), raw_read[0],raw_read[1]))

    if (loop_counter == 2):
        with open('datalog.log', 'a') as outfile:
            outfile.write("%s TEMPERATURE: %.2f\n"%(datetime.now().strftime(TIME_FORMAT), float(raw_value/10.0)))
        
    if (loop_counter == 3):
        with open('datalog.log', 'a') as outfile:
            outfile.write("%s HUMIDITY: %.2f\n"%(datetime.now().strftime(TIME_FORMAT), float(raw_value/10.0)))
    
    if (loop_counter == 0):
        with open('datalog.log', 'a') as outfile:
            outfile.write("%s SOIL: %.2f\n"%(datetime.now().strftime(TIME_FORMAT), float(raw_value/10.0)))


    time.sleep(2)  

