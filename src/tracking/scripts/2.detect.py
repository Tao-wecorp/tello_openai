#!/usr/bin/env python

import cv2
from copy import deepcopy
import numpy as np
import os
import rospy
import rospkg
rospack = rospkg.RosPack()
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Helpers
from helpers.cvlib import Detection
detection = Detection()


class Detect(object):
    def __init__(self):
        rospy.init_node('detect_node', anonymous=True)
        rate = rospy.Rate(30)
        
        self.bridge = CvBridge()
        self.frame = None

        rospy.Subscriber('/tello/image_raw', Image, self.img_callback)
        
        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)
                centroids, bboxes = detection.detect(frame)
                if len(centroids) != 0:
                    for cent in centroids:
                        cv2.circle(frame, (cent[0], cent[1]), 3, [0,0,255], -1, cv2.LINE_AA)

                cv2.imshow("", frame)
                cv2.waitKey(1)

            rate.sleep()

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        self.frame = cv_image


if __name__ == '__main__':
    try:
        Detect()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()