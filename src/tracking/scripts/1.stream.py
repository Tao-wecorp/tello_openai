#!/usr/bin/env python

import threading
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from std_msgs.msg import Empty

import av
import cv2
import numpy
import tellopy
from cv_bridge import CvBridge
import time


class Stream(object):
    def __init__(self):
        rospy.init_node('stream_node', anonymous=False)

        # ROS publishers
        self.image_pub = rospy.Publisher('tello/image_raw', Image, queue_size=1)
        self.cv_bridge = CvBridge()

         # Connect to the drone
        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60.0)
        self.drone.takeoff()
        rospy.loginfo('connected to drone')

        # Start video thread
        self.stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()

        # Yaw subscriber
        self.yaw = 0.0
        rospy.Subscriber('/tello/yaw_angle', Float64, self.yaw_callback)
        
        rospy.spin()
        rospy.on_shutdown(self.shutdown)
    
    def video_worker(self):
        container = av.open(self.drone.get_video_stream())
        rospy.loginfo('starting video pipeline')

        for frame in container.decode(video=0):
            color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            frame = deepcopy(color_mat)
            frame = cv2.resize(frame, (640,480))
            centroids, bboxes = detection.detect(frame)
            if len(centroids) != 0:
                target = centroids[0]

                xoff = int(target[0] - 320)
                distance = 100
                cmd = ""

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

            if self.stop_request.isSet():
                return

    def shutdown(self):
            self.stop_request.set()
            self.drone.land()
            self.drone.quit()
            self.drone = None

    def yaw_callback(self, data):
        print(data.data)
        self.yaw = data.data

def main():
    try:
        Stream()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()
