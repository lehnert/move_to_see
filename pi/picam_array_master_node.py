#!/usr/bin/env python

import RPi.GPIO as gpio
from picamera import PiCamera
import os
import cv2
import numpy as np
import time

import rospy
from harvey_msgs.msg import piCamArray, piCamObject

class piCameraMaster:

    def __init__(self,nCameras):

        self.nCameras = nCameras
        self.trigger = True
        self.received_data = [False]*nCameras

        self.publisher = rospy.Publisher("pi_camera_array", piCamArray, queue_size=100)
        self.subscribers = []

        #BCM based on gpio names not pin numbers
        gpio.setmode(gpio.BCM)

        #create nCamera amount of subcribers
        self.camera_data = []
        for i in range(0,self.nCameras):
            print "Initialising GPIO lines: " + str(i+3)
            gpio.setup(i+4, gpio.OUT)
            gpio.output(i+4, False)
            self.camera_data.append(piCamObject())
            self.received_data[i] = False
            #create subscriber and use i to inform callback which camera topic is subscribed
            self.subscribers.append(rospy.Subscriber("pi_camera_"+str(i),piCamObject,self.camera_callback,i))

        #create empty camera array msg with empty list of camera data objects
        self.piCamArray_msg = piCamArray()
        self.piCamArray_msg.nCameras = nCameras
        self.piCamArray_msg.piCamObjects = self.camera_data
        self.piCamArray_msg.header.frame_id = "/picamera_frame"

    def camera_callback(self,data,camera_number):
        # print "Received camera data from camera: " + str(camera_number)
        # print data
        self.piCamArray_msg.piCamObjects[camera_number] = data
        self.received_data[camera_number] = True

    def run(self):

        print "Running camera array master"
        r1 = rospy.Rate(100) # 100hz

        count = long(0)

        while not rospy.is_shutdown():
            #reset camera array data msg
            count = count + 1
            #need to trigger gpio lines
            # if self.trigger:
            # print "Triggering Cameras"
            for i in range(0,self.nCameras):
                gpio.output(i+4, True)

            #update header info
            self.piCamArray_msg.header.stamp = rospy.Time.now()
            self.piCamArray_msg.header.seq = count

            #wait for subscriber callbacks to receive data for each camera_data
            #then publish collated camera array data
            # print "recevied data flags: " + str(self.received_data)

            if(sum(self.received_data) == self.nCameras):
                print "received all camera data, publishing"
                self.publisher.publish(self.piCamArray_msg)
                self.received_data = [False]*self.nCameras #reset data array

            for i in range(0,self.nCameras):
                gpio.output(i+4, False)

            #wait for first sleep to reset GPIO pins
            r1.sleep()

if __name__ == "__main__":

    print "creating pi camera array master node"

    rospy.init_node('pi_camera_array_master')

    nCameras = rospy.get_param("~number_of_cameras",9)

    camera_array = piCameraMaster(nCameras)
    camera_array.run()
