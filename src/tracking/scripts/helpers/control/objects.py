#! /usr/bin/env python

import rospy
import time
from math import *
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Control:
    def __init__(self):
        self.yaw = 0.0  # [angle]

    def yaw(self, position):
        new_yaw = degrees(atan(float(320-position[0])/(480-position[1])))
        yaw = new_yaw + self.yaw
        self.yaw = yaw
        return new_yaw