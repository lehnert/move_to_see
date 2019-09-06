#!/usr/bin/env python
# Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved.
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
#
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
#
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!


#OLD VREP hue, sat, L parameters = 0.00, 0.5, 1.0
#OLD VREP tolerage = 0.1, 0.2, 1.0

import time
import numpy as np
import math
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2

class test():

    def __init__(self):

        self.cv_image_array = []
        self.got_image_array = []
        self.nCameras = 9

        self.bridge = CvBridge()

        self.image_sub_array = []
        for i in range(0, 9):
            print "Creating message filter sub on topic: " + "/pi_camera_"+str(i)+"/rgb_image"
            self.image_sub_array.append(message_filters.Subscriber("/pi_camera_"+str(i)+"/rgb_image", Image))

        # self.ts = message_filters.TimeSynchronizer(self.image_sub_array, 100)
        self.ats = message_filters.ApproximateTimeSynchronizer(self.image_sub_array, queue_size=1, slop=0.4)
        self.ats.registerCallback(self.imageArrayCallback)

        # self.ts.registerCallback(self.imageArrayCallback)

    def imageArrayCallback(self, image_0, image_1, image_2, image_3, image_4, image_5, image_6, image_7, image_8):
    # def imageArrayCallback(self, image_0, image_1):

        print "Got 9 Images from Camera"



        image_array = [image_0,image_1,image_2,image_3,image_4,image_5,image_6,image_7,image_8]
        #
        for i in range(0, 9):
            self.cv_image_array.append(self.bridge.imgmsg_to_cv2(image_array[i], desired_encoding="passthrough"))

            print "Image size: " + str(self.cv_image_array[i].shape[:2])

        for i in range(1, 9):

            time_diff = image_array[i].header.stamp - image_array[i-1].header.stamp
            print time_diff.to_sec()

        #return images or process them here
        # image_array = [image_0,image_1,image_2,image_3,image_4,image_5,image_6_,image_7,image_8]
        #
        # try:
        #     for i in range(0,self.nCameras):
        #         self.cv_image_array[i] = self.bridge.imgmsg_to_cv2(image_array[i], desired_encoding="passthrough")
        #         self.got_image_array[i] = True
        # except CvBridgeError as e:
        #     print(e)


if __name__=="__main__":

    rospy.init_node("test")

    t = test()

    rospy.spin()
