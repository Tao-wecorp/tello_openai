#!/usr/bin/env python

import av
import cv2
from copy import deepcopy
from math import *
import numpy as np
import os
from scipy.spatial import distance as dist
import threading
import tellopy
import time

# ros
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Empty, Float64, String
from tracking.msg import BBox, BBoxes

# helpers
from helpers.cvlib import Detection
detection = Detection()

from helpers.mars import DeepFeatures
mars = DeepFeatures()
roi_dist = 400 # To-do: dynamic
feature_dist = 0.4
neighbor_dist = 0.15
width = 640
height = 480

class Yaw(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=False)
        rate = rospy.Rate(30)

        # connect to the drone
        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60.0)
        self.drone.takeoff()
        rospy.loginfo('connected to drone')

        # start video thread
        self.stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()
        self.frame = None

        # keypress for selection
        self.prev_keypress = -1
        self.keypress = -1
        rospy.Subscriber('/keypress', String, self.key_callback)

        # yaw cmd
        self.yaw_speed = 50
        self.yaw_cmd = ""

        # tracking history
        self.tracking_bbox_features = None
        self.prev_target_cent = None
        self.prev_target_features = None
        target_id = -1
        
        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)
                frame = cv2.resize(frame, (width, height))
                centroids, bboxes = detection.detect(frame) # arrays

                if len(centroids) > 0:
                    # select target id using keypress
                    if self.keypress != -1 and self.keypress != self.prev_keypress:
                        target_id = self.keypress
                        self.tracking_bbox_features = mars.extractBBoxFeatures(frame, bboxes, target_id)
                        self.prev_target_cent = centroids[target_id]
                        self.prev_keypress = self.keypress
                    elif self.prev_target_cent is not None:
                        # centroids_roi, bboxes_roi = self.__roi(centroids, bboxes)
                        # if len(centroids_roi) > 0:

                        # extract features of bboxes
                        bboxes_features = mars.extractBBoxesFeatures(frame, bboxes)
                        features_distance = dist.cdist(self.tracking_bbox_features, bboxes_features, "cosine")[0]
                        tracking_id = self.__assignNewTrackingId(features_distance, threshold=feature_dist)

                        if tracking_id != -1:
                            print(tracking_id)
                            taeget_cent = centroids[tracking_id]
                            self.prev_target_cent = taeget_cent # for roi 
                            cv2.rectangle(frame, (taeget_cent[0]-20, taeget_cent[1]-40), (taeget_cent[0]+20, taeget_cent[1]+40), (0,0, 255), 1)

                            # xoff = int(taeget_cent[0] - width/2)
                            # self.__yaw(xoff)
                
                    i = 0
                    for cent in centroids:                   
                        cv2.circle(frame, (cent[0], cent[1]), 3, [0,0,255], -1, cv2.LINE_AA)
                        cv2.putText(frame, str(i), (cent[0], cent[1]), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0), 2)
                        i = i + 1

                cv2.imshow("", frame)
                cv2.waitKey(1)
            rate.sleep()
        
        rospy.spin()
        rospy.on_shutdown(self.shutdown)


    def key_callback(self, data):
        if data.data != "":
            self.keypress = int(data.data)
        else:
            self.keypress = -1

    def video_worker(self):
        container = av.open(self.drone.get_video_stream())
        rospy.loginfo('starting video pipeline')

        for frame in container.decode(video=0):
            self.frame = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            if self.stop_request.isSet():
                return

    #Tracking functions
    def __roi(self, centroids, bboxes):
        # Logic: 
        # 1. Only compare features of targets within centroids ROI

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

    # yaw
    def __yaw(self, xoff):
        distance = 100
        cmd = ""
        
        if xoff < -distance:
            cmd = "counter_clockwise"
        elif xoff > distance:
            cmd = "clockwise"
        else:
            if self.yaw_cmd is not "":
                getattr(self.drone, self.yaw_cmd)(0)
                self.yaw_cmd = ""

        if cmd is not self.yaw_cmd:
            if cmd is not "":
                getattr(self.drone, cmd)(self.yaw_speed)
                self.yaw_cmd = cmd

    def shutdown(self):
        self.stop_request.set()
        self.drone.land()
        self.drone.quit()
        self.drone = None


def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()