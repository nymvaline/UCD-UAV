#!/usr/bin/env python
import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')
import mraa as m
import time 
# this example will show the 'advanced' i2c functionality from python i2c
# read/write

x = m.I2c(6)
x.frequency(100000)
x.address(0b1101000)
# genreal call
# x.address(0b0000000)
while (True):
    # x.writeByte(0x8)
    d=x.read(2)
    # print str(d)
    time.sleep(2)


# x.address(0xc)

# initialise device
# if x.readReg(0xd0) != 0x55:
#   print("error")

# we want to read temperature so write 0x2e into control reg
# x.writeReg(0xf4, 0x2e)

# read a 16bit reg, obviously it's uncalibrated so mostly a useless value :)
# print(str(x.readWordReg(0xf6)))

# and we can do the same thing with the read()/write() calls if we wished
# thought I'd really not recommend it!
# while(True):
#     x.write(bytearray(b'0xf40x2e'))

#     x.writeByte(0xf6)
#     d = x.read(2)

# WARNING: python 3.2+ call
# print(str(d))
# print(int.from_bytes(d, byteorder='little'))