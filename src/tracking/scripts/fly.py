#!/usr/bin/env python

import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tello_msgs.msg import FlightData

import av
import cv2
import numpy
import tellopy
from cv_bridge import CvBridge