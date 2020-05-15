#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import cv2
from cv_bridge import CvBridge
import tf
import tf.transformations as tft
import numpy as np