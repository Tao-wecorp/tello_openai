#!/usr/bin/env python

import numpy as np
import rospy
import roslib
import subprocess
import time
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class Ctrl():
    def __init__(self):
        rospy.init_node('ctrl_node', anonymous=True)
        self.rate = rospy.Rate(30)

        self.pose_sub = rospy.Subscriber('/joy', Joy, self.ctrl_callback)

    def ctrl_callback(self, data):
        self.joy = data.buttons
        self.joy2= data.axes
        print(self.joy)
        

if __name__ == '__main__':
    ctrl = Ctrl()