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
from harvey_msgs.msg import *
import sensor_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from harvey_msgs.msg import piCamArray, piCamObject
from harvey_msgs.msg import moveToSee



import ros_realsense.srv
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from sensor_msgs.point_cloud2 import read_points, create_cloud
#import pylab as plb

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo


import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf2_geometry_msgs

class ros_interface:

    def __init__(self, number_of_cameras):

        self.type = "ROS"

        self.bridge = CvBridge()
        
        self.sceneTopic = 'realsense/rgb/image_raw'
        self.sceneDepthTopic = 'realsense/depth/image_raw'

        rospy.loginfo('Waiting for segment capsicum image service')
        service_name = '/harvey_detection/segment_capsicum'
        rospy.wait_for_service(service_name)
        self.segment_capsicum_service = rospy.ServiceProxy(service_name, harvey_msgs.srv.segmentCapsicum)
        
        rospy.loginfo('Waiting for segment capsicum image service')
        service_name = '/harvey_detection/segment_capsicum_image'
        rospy.wait_for_service(service_name)
        self.segment_capsicum_image_service = rospy.ServiceProxy(service_name, harvey_msgs.srv.segmentCapsicumImage)

        rospy.loginfo('Waiting for register depth service')
        service_name = '/realsense/register_depth'
        rospy.wait_for_service(service_name)
        self.register_depth_service = rospy.ServiceProxy(service_name, ros_realsense.srv.registerDepth)
        self.capsicum_clouds = 'NONE'

        self.got_scene = False
        

        # self.move_group = move_group
        self.min_path_completion = 0.5
        self.max_attempts = 3
        self.max_planning_time = 1.0
        self.nCameras = number_of_cameras
        self.new_camera_data = False
        
        self.tf_listener = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.publisher = rospy.Publisher("/move_to_see_data", moveToSee, queue_size=10)
        self.move_to_see_msg = moveToSee()
        self.move_to_see_msg.nCameras = number_of_cameras
        self.move_to_see_msg.header.stamp = rospy.Time.now()
        self.move_to_see_msg.header.seq = 0
        self.move_to_see_msg.header.frame_id = "/move_to_see"

        self.fixed_frame = "/pi_camera_link"

        # self.pixel_sizes = np.zeros((number_of_cameras,))
        # self.pixel_sizes_filtered = np.zeros((number_of_cameras,))
        # self.blob_centres = np.zeros((number_of_cameras,2))

        self.pixel_sizes = self.init_list_of_objects(number_of_cameras, 0)
        self.pixel_sizes_filtered = self.init_list_of_objects(number_of_cameras, 0)
        self.blob_centres = self.init_list_of_objects(number_of_cameras, 0)
        # self.pixel_sizes_filtered = [0.0]*number_of_cameras
        # self.blob_centres = [(0.0,0.0)]*number_of_cameras

        self.window_size = 5
        # self.pixel_sizes_buffer = [[0.0]*self.window_size]*number_of_cameras
        self.pixel_sizes_buffer = self.init_list_of_objects(number_of_cameras,self.window_size)
        # self.blob_centres = [[(0.0,0.0), (0.0,0.0), (0.0,0.0), (0.0,0.0), (0.0,0.0)]]*number_of_cameras

        self.camarray_sub = rospy.Subscriber("/pi_camera_array",piCamArray,self.piCameraCallback)

        self.move_pose_service_name = '/harvey_robot/move_harvey_pose_array'
        
        self.get_jacobian_service_name = '/harvey_robot/get_jacobian'
        
        self.get_manipulability_service_name = '/harvey_robot/get_manipulability'
        
        #self.move_pose_service_name = '/harvey_robot/move_harvey_pose'

        rospy.loginfo('Waiting for service %s to come online ...' % self.move_pose_service_name)

        self.move_to_named_service_name = '/harvey_robot/move_harvey_named'
        rospy.loginfo('Waiting for service %s to come online ...' % self.move_pose_service_name)

        try:
            rospy.wait_for_service(self.move_pose_service_name)
            self.move_pose_service = rospy.ServiceProxy(self.move_pose_service_name,harvey_msgs.srv.move_harvey_pose_array)
            #self.move_pose_service = rospy.ServiceProxy(self.move_pose_service_name, harvey_msgs.srv.move_harvey_pose)

            rospy.wait_for_service(self.move_to_named_service_name)
            self.move_named_service = rospy.ServiceProxy(self.move_to_named_service_name,
                                        harvey_msgs.srv.move_harvey_named)
            
            rospy.wait_for_service(self.get_jacobian_service_name)
            self.get_jacobian_service = rospy.ServiceProxy(self.get_jacobian_service_name,
                                        harvey_msgs.srv.getJacobian)
            
            rospy.wait_for_service(self.get_manipulability_service_name)
            self.get_manipulability_service = rospy.ServiceProxy(self.get_manipulability_service_name,
                                        harvey_msgs.srv.getManipulability)
            
            
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % service_name)

    def publish_data(pixel_sizes,pixel_sizes_unfiltered,ref_size, count):

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
    
    
    def getRefCameraImage(self):
        
        self.got_image = False
        self.image_sub = rospy.Subscriber("/pi_camera_4/rgb_image",Image,self.image_callback)
        
        while self.got_image == False:
            continue
            #print "Waiting for RGB image"
        
        self.image_sub.unregister()    
        return self.cv_image
    
        #self.camarray_sub = rospy.Subscriber("/pi_camera_4/rgb_image",piCamArray,self.image_callback);
        

    def image_callback(self, data):
        
        print "Got RGB Image"
        
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.got_image = True
        except CvBridgeError as e:
            print(e)
            
    def getPixelSize(self):      
        
        rospy.loginfo("getting current scene from camera...")
        scene = []
        self.getScene()
        
        res0 = self.segment_capsicum_image_service(self.scene.image,10)
        
        
        
        print "Pixel sizes: ", res0.pixel_sizes
        
        if res0.pixel_sizes == ():
            return 0.0
        else:
            return res0.pixel_sizes[0]
            
            
    def getCapsicumDepth(self):
        
        
        rospy.loginfo("getting current scene from camera...")
        scene = []
        self.getScene()
        #self.scene = self.scene

        rospy.loginfo("start segmenting scene using image...")


        image_size = (self.scene.image.height,self.scene.image.width)
        blank_mask = np.ones(image_size, np.uint8)
        blank_mask_msg = self.bridge.cv2_to_imgmsg(blank_mask, 'mono8')
     
        
        #self.seg_capsicum_image = self.bridge.imgmsg_to_cv2(res0.overlay_image, desired_encoding="passthrough")
        
        #cv2.imshow('vis_image', self.seg_capsicum_image)
        #cv2.waitKey(0)

        #create cloud from color image and depth image
        rospy.loginfo("registering color to depth image")
        #print("image size: " + str(image_size))
        res1 = self.register_depth_service(self.scene.image, self.scene.depth,
        blank_mask_msg, True, "/odom", True, Transform())


        rospy.loginfo("Segmenting capsicum image")
        # segment color cloud into capsicum segments
        # also produces bounding boxes
        res2 = self.segment_capsicum_service(res1.color_cloud, res1.camera_to_fixed_frame, 50)
        self.capsicum_clouds = res2.segmented_clouds
        
        
        
        self.capsicum_centroids = res2.cloud_centroids

        masks = np.zeros((image_size[0],image_size[1],3), np.uint8)

        if(len(res2.segmented_clouds) > 0):

            peduncle_roi_masks = []
            #Turn returned cap bounding boxes into roi bbox for peduncle
            for i in range(len(res2.bounding_boxes)):

                bbox = res2.bounding_boxes[i]
                #print bbox

            self.capsicum_clouds = self.capsicum_clouds
            self.capsicum_centroids = self.capsicum_centroids
            self.camera_to_fixed_frame_tf = res1.camera_to_fixed_frame

            top_left = (int(bbox.top_left.x),int(bbox.top_left.y))
            bottom_right = (int(bbox.bottom_right.x),int(bbox.bottom_right.y))

            cv2.rectangle(masks,top_left,bottom_right,(255,255,255),-1)
            
            

        else:
            rospy.loginfo("Capsicum scene is empty")
            return 'failed'

        image = self.bridge.imgmsg_to_cv2(self.scene.image, "bgr8")
        vis_image = np.zeros((image_size[0],image_size[1],3), np.uint8)
        cv2.addWeighted(image,1.0,masks,0.2,0.0,vis_image)
        
        #cv2.imshow('vis_image', vis_image)
        #cv2.waitKey(0)
        
        #print self.capsicum_centroids[0]
        
        return self.capsicum_centroids[0]
            
    def getScene(self):
        self.got_scene == False
        print 'creating message filter'
        atss = ApproximateTimeSynchronizer([Subscriber(self.sceneTopic, Image),
            Subscriber(self.sceneDepthTopic, Image)], 10, 0.1)
        atss.registerCallback(self.imageCallback2)
        print 'registering callback'

        rate = rospy.Rate(10) # 10hz
        while (self.got_scene == False):
            print 'Waiting for scene from camera'
            rate.sleep()

        print 'got image'
        self.got_scene = False
        return

    def imageCallback2(self,image,depth):
        # print 'got image'
        self.scene = harvey_msgs.msg.rgbd_image()
        self.scene.image = image
        self.scene.depth = depth
        self.got_scene = True

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
    
    
    def getCurrentPoseNaive(self):
        
        self.tf_listener.waitForTransform("camera_link", "harvey_base_link", rospy.Time(0), rospy.Duration(2))
        (position,orientation) = self.tf_listener.lookupTransform("harvey_base_link", "camera_link", rospy.Time(0))
        
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

    def getObjectiveFunctionValues(self):

        #this gets updated from subscriber callback
        #wait until new data arrives
        # for i in range(0,5):
        #print "Getting camera data"

        while not self.new_camera_data:
            print "Waiting for new camera data"
            time.sleep(0.1)

        self.new_camera_data = False
        # return self.pixel_sizes_filtered, self.blob_centres
        return copy.deepcopy(self.pixel_sizes_filtered), copy.deepcopy(self.blob_centres), copy.deepcopy(self.pixel_sizes)

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
        
    def getJacobian(self,joint_values):
        
        joint_state = sensor_msgs.msg.JointState()
        
        joint_state.position = joint_values
        
        response = self.get_jacobian_service("harvey_pi_camera",joint_state)
        
        print "Response: ", response
        
        rows = response.rows
        cols = response.cols
        jacobian_vector = response.jacobian
        
        
        jacobian = np.asarray(jacobian_vector)
        
        jacobian = jacobian.reshape((rows,cols))
        
        print "Jacobian: ", jacobian
        
        return jacobian
    
    
    def calcManipulability(self, jacobian):
     
        return math.sqrt(np.linalg.det(np.matmul(jacobian,jacobian.transpose())))
        
        

    def servoPoseNaive(self, pose, move_group="harvey_camera", frame_id="camera_link", velocity_scale=0.2):

        ros_pose_array = PoseArray()
        #ros_pose_array.header.frame_id = "harvey_base_link"
        ros_pose_array.header.frame_id = "harvey_base_link"
        ros_pose_array.header.stamp = rospy.Time.now()
        
        poseStamped = PoseStamped() 
        poseStamped.pose = pose
        
        """poseStamped.pose.position.x = pose[0]
        poseStamped.pose.position.y = pose[1]
        poseStamped.pose.position.z = pose[2]
        poseStamped.pose.orientation.x = pose[3]
        poseStamped.pose.orientation.y = pose[4]
        poseStamped.pose.orientation.z = pose[5]
        poseStamped.pose.orientation.w = pose[6]"""
        
        poseStamped.header.frame_id = frame_id
        ros_pose_array.header.stamp = rospy.Time.now()            
        
        try:
            #t = self.tf_listener.getLatestCommonTime("harvey_base_link", frame_id)
            
            trans = self.tfBuffer.lookup_transform("harvey_base_link", frame_id, rospy.Time(0), rospy.Duration(1.0))
            
            #trans = self.tf_listener.lookupTransform("harvey_base_link", frame_id, t)
            transformed_pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans)
            
           

 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
            print(e)
            rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
            return None

        #ros_pose = Pose()
        #ros_pose.position.x = pose[0]
        #ros_pose.position.y = pose[1]
        #ros_pose.position.z = pose[2]
        #ros_pose.orientation.x = pose[3]
        #ros_pose.orientation.y = pose[4]
        #ros_pose.orientation.z = pose[5]
        #ros_pose.orientation.w = pose[6]
        
        transformed_pose.pose.position.z = transformed_pose.pose.position.z - 0.011

        ros_pose_array.poses.append(transformed_pose.pose)

        #print ("pose: \n")
        #print transformed_pose
        #print "\n"

        # rospy.loginfo('Trying to move robot\'s %s. Goal is named pose: %s' % (self.movegroup, ros_pose))
        if self.move_pose_service(move_group, ros_pose_array, velocity_scale, self.min_path_completion, self.max_attempts, self.max_planning_time).success:
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
