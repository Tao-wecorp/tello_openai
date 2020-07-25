#!/usr/bin/env python

import av
import cv2
import numpy as np
import threading
import tellopy


class VideoStreamer():
    def __init__(self):
        self.__initVariables()
        self.__droneSetup()
        self.__videoSetup()
        self.__createThreads()
        self.main()

    def __initVariables(self):
        self.packet = None
        self.frame = None

    def __droneSetup(self):
        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60.0)

    def __videoSetup(self):
        done = False
        while not done:
            try:
                self.container = av.open(self.drone.get_video_stream())
                done = True
            except av.AVError:
                print("Trying again")
                done = False

    def __createThreads(self):
        self._frame_thread = threading.Thread(target=self.packetCallback)
        self._frame_thread.daemon = True
        self._frame_thread.start()
    
    def packetCallback(self):
        for packet in self.container.demux(video=0):
            # print("Packet demuxed {}, {}".format(type(packet), type(self.packet)))
            self.packet = (frame for frame in packet.decode())

    def main(self):
        while True:
            if self.packet is None:
                continue
            try:
                self.frame = next(self.packet)
            except StopIteration:
                continue
            frame_d = np.uint8(np.array(self.frame.to_image()))
            cv2.imshow('frame', frame_d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
       
            
if __name__ == '__main__':
    main()