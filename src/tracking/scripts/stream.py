#!/usr/bin/env python

import threading
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

import av
import cv2
import numpy
import tellopy
from cv_bridge import CvBridge


class Stream(object):
    def __init__(self):
        rospy.init_node('stream_node', anonymous=False)

        # ROS publishers
        self._image_pub = rospy.Publisher('tello/image_raw', Image, queue_size=10)
        self._cv_bridge = CvBridge()

         # Connect to the drone
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(60.0)
        rospy.loginfo('connected to drone')

        # Start video thread
        self._stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()
        
        rospy.spin()
        rospy.on_shutdown(self.shutdown)
    
    def video_worker(self):
        container = av.open(self._drone.get_video_stream())
        rospy.loginfo('starting video pipeline')

        for frame in container.decode(video=0):
            color = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

            if self._stop_request.isSet():
                return

    def shutdown(self):
            self._stop_request.set()
            self._drone.land()
            self._drone.quit()
            self._drone = None

def main():
    try:
        Stream()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()