#!/usr/bin/env python

import subprocess
import re
tasklog = open('task_list.log', 'r')
tasklist=[]

for eachline in tasklog:
	line = eachline.strip('\n').split(' ', 1)
	tasklist.append(['python', str(line[0])+'.py', line[1]])

print tasklist[0]

#call the first task
subprocess.call(tasklist[0])


