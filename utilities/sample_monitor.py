#!/usr/bin/env python

'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
from os import system
import curses

def get_param(prompt_string):
    screen.clear()
    screen.border(0)
    screen.addstr(2, 2, prompt_string)
    screen.refresh()
    input = screen.getstr(10, 10, 60)
    return input

def execute_cmd(cmd_string):
    system("clear")
    a = system(cmd_string)
    print ""
    if a == 0:
      print "Command executed correctly"
    else:
      print "Command terminated with error"
    raw_input("Press enter")
    print ""

x = 0

while (x != ord('4')):
    screen = curses.initscr()

    screen.clear()
    screen.border(0)
    screen.addstr(2, 2, "Please enter a number...")
    screen.addstr(4, 4, "1 - Add a user")
    screen.addstr(5, 4, "2 - Restart Apache")
    screen.addstr(6, 4, "3 - Show disk space")
    screen.addstr(7, 4, "4 - Exit")
    screen.refresh()

    x = screen.getch()

    if x == ord('1'):
      username = get_param("Enter the username")
      homedir = get_param("Enter the home directory, eg /home/nate")
      groups = get_param("Enter comma-separated groups, eg adm,dialout,cdrom")
      shell = get_param("Enter the shell, eg /bin/bash:")
      curses.endwin()
      execute_cmd("useradd -d " + homedir + " -g 1000 -G " + groups + " -m -s " + shell + " " + username)
    if x == ord('2'):
      curses.endwin()
      execute_cmd("apachectl restart")
    if x == ord('3'):
      curses.endwin()
      execute_cmd("df -h")

curses.endwin()
