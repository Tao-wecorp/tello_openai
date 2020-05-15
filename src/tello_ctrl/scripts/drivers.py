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


class TelloDriver(object):
    def __init__(self):
        rospy.init_node('tello_driver_node', anonymous=False)

        # ROS publishers
        self._flight_data_pub = rospy.Publisher('tello/flight_data', FlightData, queue_size=10)
        self._image_pub = rospy.Publisher('tello/image_raw', Image, queue_size=10)
        self._cv_bridge = CvBridge()

        # Connect to the drone
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(60.0)
        rospy.loginfo('connected to drone')

        # Listen to flight data messages
        self._drone.subscribe(self._drone.EVENT_FLIGHT_DATA, self.flight_data_callback)

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

    def flight_data_callback(self, event, sender, data, **args):
        flight_data = FlightData()

        flight_data.battery_percent = data.battery_percentage
        flight_data.estimated_flight_time_remaining = data.drone_fly_time_left / 10.
        flight_data.flight_mode = data.fly_mode
        flight_data.flight_time = data.fly_time
        flight_data.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
        flight_data.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
        flight_data.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.
        flight_data.altitude = -1. if data.height > 30000 else data.height / 10.
        flight_data.equipment = data.electrical_machinery_state
        flight_data.high_temperature = data.temperature_height

        self._flight_data_pub.publish(flight_data)

    def shutdown(self):
        self._stop_request.set()
        self._drone.land()
        self._drone.quit()
        self._drone = None

def main():
    try:
        TelloDriver()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()