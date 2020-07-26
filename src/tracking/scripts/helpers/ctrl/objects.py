#! /usr/bin/env python

import rospy
import time
from math import *
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Ctrl:
    def yaw(self, target, grid=100):
        xoff = int(target[0] - 320)
        cmd = ""
        if xoff < -1*gird:
            cmd = "counter_clockwise"
        elif xoff > grid:
            cmd = "clockwise"
        return cmd