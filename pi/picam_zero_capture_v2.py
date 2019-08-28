#!/usr/bin/env python

import RPi.GPIO as gpio
from picamera import PiCamera
import os
import cv2
import numpy as np
import time


import rospy
from move_to_see_msgs.msg import piCamArray, piCamObject
from sensor_msgs.msg import Image
from pi_cv_bridge import CvBridge
# from dynamic_reconfigure.server import Server


class piCamera:

    def __init__(self,camera_index, camera_topic, publish_image, publish_segment_image, detect_objects, hsv_thresholds_low, hsv_thresholds_high):

        self.createCamera([640,480],60)
        # define range of blue color in HSV
        # self.lower_red = np.array([0,115,0])
        # self.upper_red = np.array([15,255,255])
        #
        # self.lower_red2 = np.array([165,115,0])
        # self.upper_red2 = np.array([180,255,255])


        self.lower_red = np.copy(hsv_thresholds_low)
        self.upper_red =  np.copy(hsv_thresholds_high)
        #
        self.lower_red2 =  np.copy(hsv_thresholds_low)
        self.lower_red2[0] = 180 - self.upper_red[0]
        self.upper_red2 = np.array([180,255,255])

        print ('lower_red ', self.lower_red)
        print "\n"
        print ('upper_red ', self.upper_red)
        print "\n"
        print ('lower_red2 ', self.lower_red2)
        print "\n"
        print ('upper_red2 ', self.upper_red2)
        print "\n"

        self.camera_index = camera_index
        self.count = long(0)

        gpio.setmode(gpio.BCM)
        gpio.setup(self.camera_index + 4, gpio.IN)

        self.camera_topic = camera_topic
        self.publisher = rospy.Publisher(camera_topic, piCamObject, queue_size=1)
        self.publish_image = publish_image
        self.publish_segment_image = publish_segment_image

        self.detect_objects = detect_objects

        if(publish_image):
            self.bridge = CvBridge()
            self.image_publisher = rospy.Publisher(camera_topic+"/rgb_image",Image,queue_size=1)

        if(publish_segment_image):
            self.bridge = CvBridge()
            self.image_segment_publisher = rospy.Publisher(camera_topic+"/rgb_segmented_image",Image,queue_size=1)

        # self.srv = Server(defaultConfig, self.dyn_reconfigure_callback)

        #gpio.add_event_detect(self.camera_index + 4, gpio.RISING, callback=self.run)

    def createCamera(self, resolution, framerate):
        self.camera = PiCamera()
        self.camera.resolution = (resolution[0],resolution[1])
        self.camera.framerate = framerate


    def run(self,channel):

        # for i in range(0,100):
        print("Capturing Image")
	image = self.capture(self.camera)

    	#start = time.time()
        if self.detect_objects:

            object_, segmentedImage = self.detect_object(image)

            camera_object_msg = piCamObject()

            # for obj in objects:
            camera_object_msg.nObjects = len(object_) #should be 1 or 0

            if camera_object_msg.nObjects > 0:
                camera_object_msg.centre_x.append(object_["centre_x"])
                camera_object_msg.centre_y.append(object_["centre_y"])
                camera_object_msg.pixel_size.append(object_["size"])

            print "publishing camera data to master node"
            # print "camera_object_msg: " + str(camera_object_msg)

            self.publisher.publish(camera_object_msg)

        if self.publish_image:

            self.count = self.count + 1
            img_msg = self.bridge.cv2_to_imgmsg(image, "passthrough")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.seq = self.count
            img_msg.header.frame_id = self.camera_topic
            self.image_publisher.publish(img_msg)

        if self.publish_segment_image:
            img_msg = self.bridge.cv2_to_imgmsg(segmentedImage, "passthrough")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.seq = self.count
            img_msg.header.frame_id = self.camera_topic
            self.image_segment_publisher.publish(img_msg)

            # cv2.imshow("Image",image)
            # cv2.imshow("Segmented Image",segmentedImage)
            # cv2.waitKey()

#detect red object and return info such as centre and size
    def detect_object(self,image):
           # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue colors
        mask1 = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1,mask2)

        erode = cv2.erode(mask,np.ones((5,5)))
        mask = cv2.dilate(erode,np.ones((5,5)))

        # # Bitwise-AND mask and original image
        if self.publish_segment_image:
            segmentedImage = cv2.bitwise_and(image,image, mask=mask)

    	# start = time.time()
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # end = time.time()
        # print "Find Contour time: " + str(end - start)
        # cv2.drawContours(outputImage, contours, -1, (0,255,0), 3)

        object_ = []
        # centre_x = []
        # centre_y = []

        if len(contours) != 0:
            #find the biggest area
            cnt = max(contours, key = cv2.contourArea)
            pixel_size = cv2.contourArea(cnt)
            if pixel_size > 1000:
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                object_ = {'centre_x':cx,'centre_y':cy,'size':pixel_size}
                # objects.append(dict(object_))

        # for i, cnt in enumerate(contours):
        #     size = cv2.contourArea(cnt)
        #     if size > 1000:
        #         M = cv2.moments(cnt)
        #         cx = int(M['m10']/M['m00'])
        #         cy = int(M['m01']/M['m00'])
        #         object_ = {'centre_x':cx,'centre_y':cy,'size':size}
        #         objects.append(dict(object_))
                #cv2.drawContours(segmentedImage, contours, i , (0,255,0), 3)
                # print 'Found object with coordinates x,y: ' + str(cx) + ', ' + str(cy) + ' and size: ' + str(size)

        return object_, segmentedImage

    def detect_blobs(self,image):

        params = cv2.SimpleBlobDetector_Params()
        # # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 10
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector(params)
        # Detect blobs.
        keypoints = detector.detect(mask)
        print 'Found %d keypoints' % len(keypoints)

        for keypoint in keypoints:
            print 'Found blob with coordinates: ' + str(keypoint.pt) + 'and size: ' + str(keypoint.size)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(outputImage, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


        return keypoints, im_with_keypoints


    def capture(self,camera):

        width = camera.resolution.width
        height = camera.resolution.height

        output_image = np.empty((width*height*3), dtype=np.uint8)
        camera.capture(output_image, 'bgr', use_video_port=True)
        # print('Captured image')
        # print('Captured %dx%d image' % (
            # output_image.array.shape[1], output_image.array.shape[0]))
        output_image = output_image.reshape((height, width, 3))
        return output_image


if __name__ == "__main__":

    rospy.init_node('pi_camera_node')

    camera_index = rospy.get_param("~camera_index",0)
    camera_topic = rospy.get_param("~camera_topic","/pi_camera_"+str(camera_index))
    publish_image = rospy.get_param("~publish_image",True)
    publish_segment_image = rospy.get_param("~publish_segment_image",True)
    detect_objects = rospy.get_param("~detect_objects",True)


    hsv_hue_low = rospy.get_param("~hsv_hue_low",0)
    hsv_hue_high = rospy.get_param("~hsv_hue_high",15)

    hsv_sat_low = rospy.get_param("~hsv_sat_low",115)
    hsv_sat_high = rospy.get_param("~hsv_sat_high",255)

    hsv_val_low = rospy.get_param("~hsv_val_low",0)
    hsv_val_high = rospy.get_param("~hsv_val_high",255)

    hsv_thresholds_low = np.array([hsv_hue_low, hsv_sat_low, hsv_val_low])
    hsv_thresholds_high = np.array([hsv_hue_high, hsv_sat_high, hsv_val_high])


    print "creating pi camera node with camera_index: " + str(camera_index) + " and camera_topic: " + camera_topic

    camera = piCamera(camera_index, camera_topic, publish_image, publish_segment_image, detect_objects, hsv_thresholds_low, hsv_thresholds_high)

    rospy.spin()
    r = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
	camera.run()
	r.sleep()

