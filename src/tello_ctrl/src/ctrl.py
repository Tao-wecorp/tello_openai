#!/usr/bin/env python

import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from flock_msgs.msg import Flip, FlightData
import av
import cv2
import numpy
import tellopy
from cv_bridge import CvBridge

class Tello(object):
    def __init__(self):
        rospy.init_node('tello_node', anonymous=False)

        # Connect to the drone
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(60.0)
        rospy.loginfo('connected to drone')

        # Spin until interrupted
        rospy.spin()

        # Force a landing
        self._drone.land()

        # Shut down the drone
        self._drone.quit()
        self._drone = None

if __name__ == '__main__':
    tello = Tello()