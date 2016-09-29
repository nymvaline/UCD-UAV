#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import subprocess
import re
import os
import sys
import platform





tasklog = open('task_list.log', 'r')
tasklist=[]
tasklist_amount=0
tasklist_counter=0
tasklist_finish_flag=True
tasklist_env = os.environ.copy()

for eachline in tasklog:
    line = eachline.strip('\n').split(' ')
    # python TASK.py [args] [timeout in second]
    tasklist.append(['python', str(line[0])+'.py', ' '.join(line[1:-1])])
    tasklist_amount+=1
tasklog.close()

print tasklist[0]

#call the first task
# subprocess.call(' '.join(tasklist[0]), env=tasklist_env)
subprocess.Popen(tasklist[0], env=tasklist_env)
print "task executed"
# subprocess.call('', shell=True, env=tasklist_env)


