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


import time
#import cv2 as cv
#import array
import numpy as np
import math
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#import math
#import cProfile
#from PIL import Image as I
#from pyquaternion import Quaternion as pyQuat
#from collections import deque
import scipy
#import scipy.sparse.linalg
#import scipy.linalg

import rospy
import copy
import tf
import tf2_ros
import harvey_msgs.srv
import sensor_msgs

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from move_to_see_msgs.msg import piCamArray, piCamObject
from move_to_see_msgs.msg import moveToSee
#import pylab as plb

import cv2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import tf2_geometry_msgs
import threading
import rospy

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

class ros_interface:

    def __init__(self, number_of_cameras):

        self.type = "ROS"

        self.bridge = CvBridge()

        self.lock = threading.Lock()

        # self.move_group = move_group
        self.min_path_completion = 0.5
        self.max_attempts = 1
        self.max_planning_time = 0.1
        self.nCameras = number_of_cameras
        self.new_camera_data = False

        self.tf_listener = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.publisher = rospy.Publisher("/move_to_see_data", moveToSee, queue_size=10)

        self.ur_speed_publisher = rospy.Publisher("/ur_driver/joint_speed", JointTrajectory, queue_size=1)
        self.lift_vel_publisher = rospy.Publisher("/lift_vel_command", Float32, queue_size=1)

        self.move_to_see_msg = moveToSee()
        self.move_to_see_msg.nCameras = number_of_cameras
        self.move_to_see_msg.header.stamp = rospy.Time.now()
        self.move_to_see_msg.header.seq = 0
        self.move_to_see_msg.header.frame_id = "/move_to_see"

        self.max_contour_size = rospy.get_param("~max_contour_size",500)

        hsv_hue_low = rospy.get_param("~hsv_hue_low",0)
        hsv_hue_high = rospy.get_param("~hsv_hue_high",12)
        hsv_sat_low = rospy.get_param("~hsv_sat_low",120)
        hsv_sat_high = rospy.get_param("~hsv_sat_high",255)
        hsv_val_low = rospy.get_param("~hsv_val_low",50)
        hsv_val_high = rospy.get_param("~hsv_val_high",255)

        self.normalise_pixels = rospy.get_param("~normalise_pixels",True)

        self.image_width = rospy.get_param("~image_width",224)
        self.image_height =  rospy.get_param("~image_height",224)

        hsv_thresholds_low = np.array([hsv_hue_low, hsv_sat_low, hsv_val_low])
        hsv_thresholds_high = np.array([hsv_hue_high, hsv_sat_high, hsv_val_high])

        self.lower_red = np.copy(hsv_thresholds_low)
        self.upper_red =  np.copy(hsv_thresholds_high)
        #
        self.lower_red2 =  np.copy(hsv_thresholds_low)
        self.lower_red2[0] = 180 - self.upper_red[0]
        self.upper_red2 = np.array([180,255,255])

        self.fixed_frame = "/pi_camera_link"

        self.cv_image_array = []
        self.got_image_array = []

        #self.images = []
        for i in range(0,self.nCameras):
            self.cv_image_array.append(0)
            self.got_image_array.append(0)

        self.pixel_sizes = self.init_list_of_objects(number_of_cameras, 0)
        self.pixel_sizes_filtered = self.init_list_of_objects(number_of_cameras, 0)
        self.blob_centres = self.init_list_of_objects(number_of_cameras, 0)

        self.window_size = 5
        self.pixel_sizes_buffer = self.init_list_of_objects(number_of_cameras,self.window_size)

        self.image_sub_array = []
        for i in range(0, self.nCameras):
            self.image_sub_array.append(message_filters.Subscriber("/pi_camera_"+str(i)+"/rgb_image", Image))

        # self.ts = message_filters.TimeSynchronizer(self.image_sub_array, 10)
        # self.ts.registerCallback(self.imageArrayCallback)
        self.ats = message_filters.ApproximateTimeSynchronizer(self.image_sub_array, queue_size=1, slop=0.4)
        self.ats.registerCallback(self.imageArrayCallback)


        self.move_pose_service_name = '/harvey_robot/move_harvey_pose_array'
        self.move_to_named_service_name = '/harvey_robot/move_harvey_named'
        self.get_jacobian_service_name = '/harvey_robot/get_jacobian'
        self.get_manipulability_service_name = '/harvey_robot/get_manipulability'

        self.joint_state_topic = '/joint_states'

        rospy.loginfo('Waiting for service %s to come online ...' % self.get_jacobian_service_name)

        try:
            rospy.loginfo('Waiting for service %s to come online ...' % self.move_pose_service_name)
            rospy.wait_for_service(self.move_pose_service_name)
            self.move_pose_service = rospy.ServiceProxy(self.move_pose_service_name,harvey_msgs.srv.move_harvey_pose_array)
            #self.move_pose_service = rospy.ServiceProxy(self.move_pose_service_name, harvey_msgs.srv.move_harvey_pose)

            rospy.loginfo('Waiting for service %s to come online ...' % self.move_to_named_service_name)
            rospy.wait_for_service(self.move_to_named_service_name)
            self.move_named_service = rospy.ServiceProxy(self.move_to_named_service_name,
                                        harvey_msgs.srv.move_harvey_named)

            rospy.loginfo('Waiting for service %s to come online ...' % self.move_to_named_service_name)
            rospy.wait_for_service(self.get_jacobian_service_name)
            self.get_jacobian_service = rospy.ServiceProxy(self.get_jacobian_service_name,
                                       harvey_msgs.srv.getJacobian)

            self.joint_state_sub = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)

            #rospy.wait_for_service(self.get_manipulability_service_name)
            #self.get_manipulability_service = rospy.ServiceProxy(self.get_manipulability_service_name,
            #                            harvey_msgs.srv.getManipulability)

            #self.image_sub = []
            #for i in range(0,self.nCameras):
            #    self.image_sub.append(rospy.Subscriber("/pi_camera_"+str(i)+"/rgb_segmented_image",Image,self.image_callback, i))

        except rospy.ServiceException, e:
            rospy.logerr('Service %s not available. Restart and try again.'% e)

    def joint_state_cb(self,data):

        self.joint_state = data


    def publish_data(self,pixel_sizes,pixel_sizes_unfiltered,ref_size, count):

        self.move_to_see_msg.header.stamp = rospy.Time.now()
        self.move_to_see_msg.header.seq = self.count
        self.move_to_see_msg.header.frame_id = "/move_to_see"
        self.move_to_see_msg.pixel_sizes = pixel_sizes
        self.move_to_see_msg.pixel_sizes_unfiltered = pixel_sizes_unfiltered

        print ("pixel sizes: \n")
        print (np.array(pixel_sizes).reshape((3,3)))
        print "\n"
        self.move_to_see_msg.ref_pixel_size = self.ref_size


    def init_list_of_objects(self,size1, size2):
        list_of_objects = list()
        for i in range(0,size1):
            list_of_objects.append( list() )
        if size2 > 0:
            for i in range(0,size1):
                for j in range(0,size2):
                    # print list_of_objects
                    list_of_objects[i].append(0.0)
        return list_of_objects

    def publishSpeedCommands(self,velocity):

        jt_msg = JointTrajectory()
        jt_msg.header.frame_id = 'harvey_base_link'
        jt_msg.header.stamp = rospy.Time.now()
        jt_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
  'wrist_2_joint', 'wrist_3_joint']

        new_vel_point = JointTrajectoryPoint()

        print velocity.shape
        new_vel_point.velocities = velocity[1:7,0].tolist()

        print new_vel_point.velocities

        jt_msg.points.append(new_vel_point)

        lift_vel_msg = Float32(velocity[0,0])

        self.lift_vel_publisher.publish(lift_vel_msg)

        print jt_msg
        self.ur_speed_publisher.publish(jt_msg)




    def getCameraImage(self, id):

        self.got_image = False
        #self.image_sub = rospy.Subscriber("/pi_camera_"+str(id)+"/rgb_image",Image,self.image_callback,id)
        #self.image_seg_sub = rospy.Subscriber("/pi_camera_"+str(id)+"/rgb_segmented_image",Image,self.image_callback,id)

        while self.got_image == False:
            continue
            #print "Waiting for RGB image"

        #self.image_sub.unregister()
        return self.cv_image_array[i]


    def getRefCameraImage(self):

        self.got_image = False
        self.image_sub = rospy.Subscriber("/pi_camera_4/rgb_image",Image,self.image_callback,4)

        while self.got_image == False:
            continue
            #print "Waiting for RGB image"

        self.image_sub.unregister()
        return self.cv_image_array[4]

        #self.camarray_sub = rospy.Subscriber("/pi_camera_4/rgb_image",piCamArray,self.image_callback);


    def image_callback(self, data, id):

        print "Got RGB Image"

        try:
            self.cv_image_array[id] = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.got_image = True
        except CvBridgeError as e:
            print(e)

    def imageArrayCallback(self, image_0, image_1, image_2, image_3, image_4, image_5, image_6, image_7, image_8):

        # print "Got 9 Images from Camera"
        #return images or process them here
        image_array = [image_0,image_1,image_2,image_3,image_4,image_5,image_6,image_7,image_8]

        self.lock.acquire()

        try:
            for i in range(0,self.nCameras):
                self.cv_image_array[i] = cv2.flip( self.bridge.imgmsg_to_cv2(image_array[i], desired_encoding="passthrough"), -1 )
                # self.cv_image_array[i] = cv2.flip( self.bridge.imgmsg_to_cv2(image_array[i], desired_encoding="passthrough"), 0 ) #pre experiment training
                self.got_image_array[i] = True
        except CvBridgeError as e:
            print(e)

        self.new_camera_data = True

        self.lock.release()

    # def getCameraImages(self):
    #
    #     if(sum(self.got_image_array) == self.nCameras):
    #         return self.cv_image_array

    def getCameraPositions(self):

        camera_positions = np.zeros([3,self.nCameras])
        camera_orientations = np.zeros([4,self.nCameras])
        camera_pose = np.zeros([7,self.nCameras])

        for idx in range(0,self.nCameras):

            self.tf_listener.waitForTransform("/pi_camera_"+str(idx), self.fixed_frame, rospy.Time(), rospy.Duration(0.5))
            (position,rotation) = self.tf_listener.lookupTransform(self.fixed_frame, "/pi_camera_"+str(idx), rospy.Time(0))
            #trans.x trans.y trans.z, rot.x rot.y rot.z rot.w

            camera_positions[0,idx] = position[0]
            camera_positions[1,idx] = position[1]
            camera_positions[2,idx] = position[2]

            camera_orientations[0,idx] = rotation[0]
            camera_orientations[1,idx] = rotation[1]
            camera_orientations[2,idx] = rotation[2]
            camera_orientations[3,idx] = rotation[3]

            camera_pose[0,idx] = position[0]
            camera_pose[1,idx] = position[1]
            camera_pose[2,idx] = position[2]
            camera_pose[3,idx] = rotation[0]
            camera_pose[4,idx] = rotation[1]
            camera_pose[5,idx] = rotation[2]
            camera_pose[6,idx] = rotation[3]

        return camera_positions,np.array(camera_orientations), camera_pose

    def getCurrentPose(self):

        self.tf_listener.waitForTransform("pi_camera_link", "harvey_base_link", rospy.Time(0), rospy.Duration(2))
        (position,orientation) = self.tf_listener.lookupTransform("harvey_base_link", "pi_camera_link", rospy.Time(0))

        #res, transform = vrep.simxGetObjectMatrix(self.clientID,self.ee_handle,self.base_frame_handle,vrep.simx_opmode_oneshot_wait)
        return position+orientation

    def servoTargetCamera(self, target_camera, step_size):

        inInts=target_camera
        inFloats=[step_size]
        emptyBuff = bytearray()
        # res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movetoCamera',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        # print "retInts: ", retInts
        if(retInts[0] == 1):
            print ('servoing failed')
            return False
        else:
            return True


    def detect_objects(self, image):
           # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        hsv = cv2.GaussianBlur(hsv,(5,5),0)

        # Threshold the HSV image to get only blue colors
        mask1 = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1,mask2)

        erode = cv2.erode(mask,np.ones((5,5)))
        mask = cv2.dilate(erode,np.ones((5,5)))

        segmentedImage = cv2.bitwise_and(image,image, mask=mask)

        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        object_ = []

        if len(contours) != 0:
            #find the biggest area
            cnt = max(contours, key = cv2.contourArea)
            pixel_size = cv2.contourArea(cnt)
            if pixel_size > self.max_contour_size:
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                object_ = {'centre_x':cx,'centre_y':cy,'size':pixel_size,'contour':cnt}

                cv2.drawContours(segmentedImage, [cnt], 0 , (0,255,0), 3)
                # objects.append(dict(object_))


        return object_, segmentedImage

    def getObjectiveFunctionValues(self):

        while not self.new_camera_data:
            print "Waiting for new camera data"
            time.sleep(0.1)

        segmentedImage_array = []

        self.lock.acquire()

        objects = []

        for i in range(0,self.nCameras):


            object, image = self.detect_objects(self.cv_image_array[i])
            objects.append(object)
            segmentedImage_array.append(image)

            if len(object) > 0:

                self.blob_centres[i] = (object['centre_x'], object['centre_y'])

                #divide by maximum pixel size
                if(self.normalise_pixels):
                    self.pixel_sizes[i] = object['size']/float(self.image_width*self.image_height)
                else:
                    self.pixel_sizes[i] = object['size']

                #keep a sliding window of sizes for filtering
                self.pixel_sizes_buffer[i].append(copy.deepcopy(self.pixel_sizes[i]))
                del self.pixel_sizes_buffer[i][0]

                self.pixel_sizes_filtered[i] = sum(self.pixel_sizes_buffer[i])/self.window_size
            else:

                # self.pixel_sizes[i] = 0
                # self.pixel_sizes_buffer[i].append(0)
                # del self.pixel_sizes_buffer[i][0]
                #
                # self.pixel_sizes_filtered[i] = sum(self.pixel_sizes_buffer[i])/self.window_size

                print "No object found in image, setting pixel size to 0"

        self.new_camera_data = False

        row_1 = np.hstack((self.cv_image_array[2], self.cv_image_array[1], self.cv_image_array[0]))
        row_2 = np.hstack((self.cv_image_array[5], self.cv_image_array[4], self.cv_image_array[3]))
        row_3 = np.hstack((self.cv_image_array[8], self.cv_image_array[7], self.cv_image_array[6]))
        image_matrix = np.vstack((row_1, row_2, row_3))

        seg_row_1 = np.hstack((segmentedImage_array[2], segmentedImage_array[1], segmentedImage_array[0]))
        seg_row_2 = np.hstack((segmentedImage_array[5], segmentedImage_array[4], segmentedImage_array[3]))
        seg_row_3 = np.hstack((segmentedImage_array[8], segmentedImage_array[7], segmentedImage_array[6]))
        seg_image_matrix = np.vstack((seg_row_1, seg_row_2, seg_row_3))

        cv2.imshow("Images", image_matrix)
        cv2.imshow("Segmented Images", seg_image_matrix)

        cv2.waitKey(1)

        ret_images = copy.deepcopy(self.cv_image_array)

        self.lock.release()

        # return self.pixel_sizes_filtered, self.blob_centres
        return copy.deepcopy(self.pixel_sizes_filtered), copy.deepcopy(self.blob_centres), copy.deepcopy(self.pixel_sizes), ret_images, objects

    # def moving_average_filter(self, data):
    #
    #
    #
    #     N = 3
    #     cumsum, moving_aves = [0], []
    #
    #     for i, x in enumerate(data, 1):
    #         cumsum.append(cumsum[i-1] + x)
    #         if i>=N:
    #             moving_ave = (cumsum[i] - cumsum[i-N])/N
    #             #can do stuff with moving_ave here
    #             moving_aves.append(moving_ave)
    def running_mean(self, x, N):
        cumsum = np.cumsum(np.insert(x, 0, 0))
        return (cumsum[N:] - cumsum[:-N]) / N



    def moving_average(self, values, window):
        weights = np.repeat(1.0, window)/window
        mv_avg = np.convolve(values, weights, 'valid')
        return mv_avg

    def piCameraCallback(self,data):


        # print "Got Camera Data"
        self.new_camera_data = True
        #
        # print "Pixel size buffer for camera : "
        # print (self.pixel_sizes_buffer)

        for i in range(0,data.nCameras):

            camObject = data.piCamObjects[i]

            # print str(i)
            # print camObject

            if camObject.nObjects > 0:
                self.blob_centres[i] = (camObject.centre_x[0], camObject.centre_y[0])

                self.pixel_sizes[i] = camObject.pixel_size[0]/float(640*480)
                # print "Pixel size " + str(i) + " : "
                # print self.pixel_sizes[i]

                #keep a sliding window of sizes for filtering
                self.pixel_sizes_buffer[i].append(copy.deepcopy(self.pixel_sizes[i]))
                del self.pixel_sizes_buffer[i][0]

                # self.pixel_sizes_buffer[i].appendleft(self.pixel_sizes[i])
                # self.pixel_sizes_buffer[i].pop()
                # print "Pixel size buffer for camera " + str(i) + " : "
                # print (self.pixel_sizes_buffer[i])

                self.pixel_sizes_filtered[i] = sum(self.pixel_sizes_buffer[i])/self.window_size



                # self.pixel_sizes_filtered[i] = self.running_mean(self.pixel_sizes_buffer[i],self.window_size)

            # del self.blob_centres[i][i][0]
            # self.blob_centres[i].append(self.blob_centres[i])

            # self.blob_centres_sizes_filtered[i] = self.moving_average(self.pixel_sizes_buffer[i],5)

            # if camObject.nObjects > 0:
            #     # if camObject.nObjects > 1:
            #     #sort pixel size ascending
            #     indices = np.argsort(camObject.pixel_size)
            #     largest_index = indices[camObject.nObjects-1]
            #     #use index 0 as detected object (future work to be sorted if multple objects)
            #     self.blob_centres[i] = (camObject.centre_x[largest_index],camObject.centre_y[largest_index])
            #     self.pixel_sizes[i] = camObject.pixel_size[largest_index]/float(640*480)


    def movetoNamedPose(self, move_group, named_pose, velocity_scale):
        if self.move_named_service(move_group, named_pose, velocity_scale, self.max_attempts, self.max_planning_time).success:
            return True
        else:
            print ('move to named pose failed')
            return False


    def getManipulability(self, camera_poses):

        camera_pose_array = PoseArray()
        camera_pose_array.header.frame_id = "harvey_base_link"
        camera_pose_array.header.stamp= rospy.Time.now()
        pose = Pose()

        for idx in range(0,self.nCameras):

            poseStamped = PoseStamped()
            pose = camera_poses[:,idx]

            poseStamped.pose.position.x = pose[0]
            poseStamped.pose.position.y = pose[1]
            poseStamped.pose.position.z = pose[2]
            poseStamped.pose.orientation.x = pose[3]
            poseStamped.pose.orientation.y = pose[4]
            poseStamped.pose.orientation.z = pose[5]
            poseStamped.pose.orientation.w = pose[6]

            try:

                transformed_pose = Pose()

                #self.tf_listener.waitForTransform("/pi_camera_"+str(idx), self.fixed_frame, rospy.Time(), rospy.Duration(0.5))
                self.tf_listener.waitForTransform("harvey_base_link","/pi_camera_"+str(idx), rospy.Time(0), rospy.Duration(0.5))
                (position,rotation) = self.tf_listener.lookupTransform("harvey_base_link", "/pi_camera_"+str(idx), rospy.Time(0))

                transformed_pose.position.x = position[0]
                transformed_pose.position.y = position[1]
                transformed_pose.position.z = position[2]
                transformed_pose.orientation.x = rotation[0]
                transformed_pose.orientation.y = rotation[1]
                transformed_pose.orientation.z = rotation[2]
                transformed_pose.orientation.w = rotation[3]

                #trans = self.tfBuffer.lookup_transform("harvey_base_link", "pi_camera_"+str(idx), rospy.Time(0), rospy.Duration(1.0))
                #transformed_pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans)

                print "Transformed pose: ", transformed_pose, " number: ", str(idx)

                camera_pose_array.poses.append(transformed_pose)


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
                print(e)
                rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
                return None

        response = self.get_manipulability_service(camera_pose_array)

        return response.manipulability


    def getJacobian(self):

        # joint_state = sensor_msgs.msg.JointState()
        #
        # joint_state.position =
        print self.joint_state

        response = self.get_jacobian_service("harvey_pi_camera",self.joint_state)

        # print "Response: ", response

        rows = response.rows
        cols = response.cols
        jacobian_vector = response.jacobian


        jacobian = np.asarray(jacobian_vector)

        jacobian = jacobian.reshape((cols,rows)).transpose()

        print jacobian.shape

        print "Jacobian: \n"
        print(np.matrix(jacobian))
        # print('\n'.join([''.join(['{:4}'.format(item) for item in row]) for row in jacobian]))

        return jacobian


    def calcManipulability(self, jacobian):

        return math.sqrt(np.linalg.det(np.matmul(jacobian,jacobian.transpose())))


    def servoCamera(self,velocity,move_group="harvey_pi_camera", frame_id="pi_camera_link", velocity_scale=0.2):

        jacobian = self.getJacobian()

        #test forward jacobian
        # q_dot = np.array([0.1,0,0,0,0,0,0])
        # x_dot = np.matmul(jacobian,q_dot)
        # print "xdot: \n", x_dot

        # q = np.array(self.joint_states.position)
        # q = q.reshape((7,1))


        inv_jacobian = np.linalg.pinv(jacobian)

        condition_jacobian = np.linalg.cond(jacobian)
        condition_inv_jacobian = np.linalg.cond(inv_jacobian)

        # inv_jacobian2 = scipy.linalg.pinv(jacobian)
        # inv_jacobian3 = scipy.linalg.pinv2(jacobian)

        print "Inverse Jacobian: \n"
        print(np.matrix(inv_jacobian))

        q_dot = np.matmul(inv_jacobian,velocity)

        print "q_dot: \n", q_dot

        print "jacobian condition: ", condition_jacobian, "\n"
        print "jacobian inv condition: ", condition_inv_jacobian, "\n"

        self.publishSpeedCommands(q_dot)


    def servoPose(self, pose, move_group="harvey_pi_camera", frame_id="pi_camera_link", velocity_scale=0.2):

        ros_pose_array = PoseArray()
        #ros_pose_array.header.frame_id = "harvey_base_link"
        ros_pose_array.header.frame_id = "harvey_base_link"
        ros_pose_array.header.stamp = rospy.Time.now()



        poseStamped = PoseStamped()

        poseStamped.pose.position.x = pose[0]
        poseStamped.pose.position.y = pose[1]
        poseStamped.pose.position.z = pose[2]
        poseStamped.pose.orientation.x = pose[3]
        poseStamped.pose.orientation.y = pose[4]
        poseStamped.pose.orientation.z = pose[5]
        poseStamped.pose.orientation.w = pose[6]

        poseStamped.header.frame_id = frame_id
        ros_pose_array.header.stamp = rospy.Time.now()

        t = time.time()
        try:
            #t = self.tf_listener.getLatestCommonTime("harvey_base_link", frame_id)

            trans = self.tfBuffer.lookup_transform("harvey_base_link", frame_id, rospy.Time(0), rospy.Duration(0.1))

            #trans = self.tf_listener.lookupTransform("harvey_base_link", frame_id, t)
            transformed_pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans)





        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
            print(e)
            rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
            return None

        dt = time.time() - t
        print "Time to find transform: ", dt

        #ros_pose = Pose()
        #ros_pose.position.x = pose[0]
        #ros_pose.position.y = pose[1]
        #ros_pose.position.z = pose[2]
        #ros_pose.orientation.x = pose[3]
        #ros_pose.orientation.y = pose[4]
        #ros_pose.orientation.z = pose[5]
        #ros_pose.orientation.w = pose[6]

        transformed_pose.pose.position.z = transformed_pose.pose.position.z - 0.009

        ros_pose_array.poses.append(transformed_pose.pose)

        #print ("pose: \n")
        #print transformed_pose
        #print "\n"
        t = time.time()
        print "Velocity of move: ", velocity_scale
        # rospy.loginfo('Trying to move robot\'s %s. Goal is named pose: %s' % (self.movegroup, ros_pose))
        if self.move_pose_service(move_group, ros_pose_array, velocity_scale, self.min_path_completion, self.max_attempts, self.max_planning_time).success:
            dt = time.time() - t
            print "Time to perform service call: ", dt
            return True
        else:
            print ('servoing failed')
            return False

        # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


        # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


    """def servoPose(self, pose, move_group="harvey_pi_camera", frame_id="pi_camera_link", velocity_scale=0.2):

            ros_pose_stamped = PoseStamped()
            ros_pose_stamped.header.frame_id = frame_id
            ros_pose_stamped.header.stamp = rospy.Time.now()

            ros_pose = Pose()
            ros_pose.position.x = pose[0]
            ros_pose.position.y = pose[1]
            ros_pose.position.z = pose[2]
            ros_pose.orientation.x = pose[3]
            ros_pose.orientation.y = pose[4]
            ros_pose.orientation.z = pose[5]
            ros_pose.orientation.w = pose[6]

            ros_pose_stamped.pose = ros_pose

            print ("pose: \n")
            print ros_pose
            print "\n"add_two_ints

            # rospy.loginfo('Trying to move robot\'s %s. Goal is named pose: %s' % (self.movegroup, ros_pose))
            if self.move_pose_service(move_group, ros_pose_stamped, velocity_scale, self.max_attempts, self.max_planning_time).success:
                return True
            else:
                print ('servoing failed')
                return False"""

    def servoPoseEuler(self, pose):

        inInts=[]
        inFloats= pose
        emptyBuff = bytearray()
        # res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movePoseEuler',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        # print "retInts: ", retInts
        if(retInts[0] == 0):
            return True
        else:
            print ('servoing failed')
            return False
        # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


        # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
