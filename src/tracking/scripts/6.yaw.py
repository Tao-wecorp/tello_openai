#!/usr/bin/env python

import threading
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from std_msgs.msg import Empty

import av
import cv2
from copy import deepcopy
import numpy
import tellopy
from cv_bridge import CvBridge
import time

# Helpers
from helpers.cvlib import Detection
detection = Detection()


class Stream(object):
    def __init__(self):
        rospy.init_node('stream_node', anonymous=False)
        rate = rospy.Rate(30)

        # ROS publishers
        self.image_pub = rospy.Publisher('tello/image_raw', Image, queue_size=1)
        self.cv_bridge = CvBridge()

        # Connect to the drone
        self.tracking = False
        self.speed = 50
        self.track_cmd = ""

        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60.0)
        self.drone.takeoff()
        rospy.loginfo('connected to drone')

        # Start video thread
        self.stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()

        self.frame = None
        
        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)
                frame = cv2.resize(frame, (640,480))
                centroids, bboxes = detection.detect(frame)
                if len(centroids) != 0:
                    target = centroids[0]
                    xoffset = int(target[0] - 320)
                    yoffset = int(240 - target[1])

                    distance = 100
                    cmd = ""
                    if self.tracking:
                        if xoff < -distance:
                            cmd = "counter_clockwise"
                        elif xoff > distance:
                            cmd = "clockwise"
                        else:
                            if self.track_cmd is not "":
                                getattr(self.drone, self.track_cmd)(0)
                                self.track_cmd = ""

                    if cmd is not self.track_cmd:
                        if cmd is not "":
                            print("track command:", cmd)
                            getattr(self.drone, cmd)(self.speed)
                            self.track_cmd = cmd


                    cv2.circle(frame, (target[0], target[1]), 3, [0,0,255], -1, cv2.LINE_AA)

                cv2.imshow("", frame)
                cv2.waitKey(1)

            rate.sleep()
        
        rospy.spin()
        rospy.on_shutdown(self.shutdown)
    
    def video_worker(self):
        container = av.open(self.drone.get_video_stream())
        rospy.loginfo('starting video pipeline')

        for frame in container.decode(video=0):
            self.frame = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            if self.stop_request.isSet():
                return

    def shutdown(self):
            self.stop_request.set()
            self.drone.land()
            self.drone.quit()
            self.drone = None


def main():
    try:
        Stream()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()