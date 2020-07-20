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
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

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