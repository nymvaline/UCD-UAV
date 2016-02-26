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
dev.frequency(1000000)

def transferAndWait(c):
    r = dev.writeByte(c)
    time.sleep(0.00005)
    print "Now I read {}".format(c)
    return r

loop_counter=0

while(True):
     # send test string
    if (loop_counter == 0):
        txbuf=bytearray("STATUS\0".encode('ascii'))
        dev.write(txbuf)
        loop_counter+=1
    if (loop_counter == 1):
        txbuf=bytearray("TEMPERATURE\0".encode('ascii'))
        dev.write(txbuf)
        loop_counter+=1
    if (loop_counter == 2):
        txbuf=bytearray("MOISTURE\0".encode('ascii'))
        dev.write(txbuf)
        loop_counter+=1
    if (loop_counter == 3):
        txbuf=bytearray("SOIL\0".encode('ascii'))
        dev.write(txbuf)
        loop_counter+=1

    time.sleep(0.1)

    # Read 0x7f
    c=transferAndWait(0xff)
    # Read payload Length
    c=transferAndWait(0xff)
    msg_len=int(c)
    print "msg length is: {}".format(msg_len)
    raw_read=0
    for i in range(0,msg_len):
        c=transferAndWait(0xff);
        print "0x%x" % int(c)
        print " "
        raw_read=(raw_read<<8)&0xff00 | int(c)&0xff;

    if (loop_counter == 2):
        print "Now TEMPERATURE is: {}".format(float(raw_read/10.0))
        
    if (loop_counter == 3):
        print"Now HUMIDITY is: {}".format(float(raw_read/10.0))
    
    if (loop_counter == 0):
        print"Now SOIL is: ".format(float(raw_read/10.0))

    print "\n"

    time.sleep(2)  

