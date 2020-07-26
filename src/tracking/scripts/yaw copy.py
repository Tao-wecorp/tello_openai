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

from helpers.mars import DeepFeatures
mars = DeepFeatures()
roi_dist = 400 # To-do: dynamic
feature_dist = 0.4
neighbor_dist = 0.15


class Stream(object):
    def __init__(self):
        rospy.init_node('stream_node', anonymous=False)
        rate = rospy.Rate(30)

        # ROS publishers
        self.image_pub = rospy.Publisher('tello/image_raw', Image, queue_size=1)
        self.cv_bridge = CvBridge()

        # Connect to the drone
        self.speed = 0
        self.track_cmd = ""

        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60.0)
        # self.drone.takeoff()
        rospy.loginfo('connected to drone')

        # Start video thread
        self.stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()

        self.frame = None
        self.tracking_bbox_features = None
        self.prev_target_cent = None
        self.prev_target_features = None

        frame_count = 0
        target_id = 0
        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)
                # frame = cv2.resize(frame, (640,480))
                centroids, bboxes = detection.detect(frame)

                if len(centroids) != 0:
                    break

                if frame_count == 0:
                    self.tracking_bbox_features = mars.extractBBoxFeatures(frame, bboxes, target_id)
                    self.prev_target_cent = centroids[target_id]
                else:
                    centroids_roi, bboxes_roi = self.__roi(centroids, bboxes)

                    if len(centroids_roi) > 0:
                        # extract features of bboxes
                        bboxes_features = mars.extractBBoxesFeatures(frame, bboxes_roi)
                        features_distance = dist.cdist(self.tracking_bbox_features, bboxes_features, "cosine")[0]
                        tracking_id = self.__assignNewTrackingId(features_distance, threshold=feature_dist)

                        if tracking_id != -1:
                            taeget_cent = centroids_roi[tracking_id]
                            self.prev_target_cent = taeget_cent
                            cv2.rectangle(frame, (taeget_cent[0]-20, taeget_cent[1]-40), (taeget_cent[0]+20, taeget_cent[1]+40), (255,0,0), 1)
                            cv2.putText(frame, str(frame_count), (taeget_cent[0]-20, taeget_cent[1]-40), cv2.FONT_HERSHEY_PLAIN, 10, (0,0,255), 3)

                frame_count = frame_count + 1

                xoff = int(taeget_cent[0] - 320)
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

    #Tracking functions
    def __roi(self, centroids, bboxes):
        # Logic: 
        # Only compare features of targets within centroids ROI

        centroids_dist = np.array(abs(centroids[:, [0]] - self.prev_target_cent[0])).flatten()
        position_roi = np.where(centroids_dist < roi_dist)[0]
        centroids_roi = centroids[position_roi, :]
        bboxes_roi = bboxes[position_roi, :]
        return centroids_roi, bboxes_roi

    def __assignNewTrackingId(self, distance, threshold):
        # Logic: 
        # 1. If detect only one and the distance is less than 0.3, assign id;
        # 2. If detect more than one, but the first two closest distances' difference is lesss than 0.1, don't assign id;
        # 3. if the first two closest distances' difference is more than 0.1, and the closest distance is less than 0.3, assign id; 

        tracking_id = -1
        dist_sort = np.sort(distance)
        if len(dist_sort) == 1:
            if distance[0] < threshold:
                tracking_id = 0
        else:
            if (dist_sort[1]-dist_sort[0]) < neighbor_dist:
                tracking_id = -1
            else:
                min_dist = np.argsort(distance.min(axis=0))
                min_position = np.where(min_dist==0)
                if distance[min_position[0][0]] < threshold:
                    tracking_id = min_position[0][0]

        return tracking_id

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