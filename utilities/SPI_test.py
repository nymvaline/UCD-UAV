#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')

import mraa as m
import random as rand
import time

dev = m.Spi(0)




dev = m.Spi(0)
print "SPI mode is: {}".format(dev.mode(0))
dev.frequency(1000000)

while(True):
	txbuf = bytearray("HELLO\n".encode('ascii'))
	# txbuf[0] = 'A'
	# txbuf[1] = 'W'
	# txbuf[2] = 'C'
	# txbuf[3] = 'e'
	# txbuf[4] = '\n'

	dev.write(txbuf)
	print "Tx sent: {}".format(txbuf)
	time.sleep(0.1)
	


# while(True):
#   txbuf = bytearray(4)
#   for y in range(0,4):
#     txbuf[y] = rand.randrange(0, 256)
#   rxbuf = dev.write(txbuf)
#   #if rxbuf != txbuf:
#     #print("We have an error captain!")
#     ##break
#     #exit(1)
#   time.sleep(0.2)






