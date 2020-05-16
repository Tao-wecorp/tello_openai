#!/usr/bin/env python

from tello import Tello
import sys
from datetime import datetime
import time


tello = Tello()

command = "command"
command = command.rstrip()
tello.send_command(command)
time.sleep(1)

command = "takeoff"
command = command.rstrip()
tello.send_command(command)
time.sleep(1)

command = "ccw 180"
command = command.rstrip()
tello.send_command(command)
time.sleep(1)

command = "land"
command = command.rstrip()
tello.send_command(command)
time.sleep(1)