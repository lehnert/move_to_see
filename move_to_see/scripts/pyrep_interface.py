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
import math
import numpy as np
import cv2

from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.harvey import Harvey
from pyrep.objects.object import Object
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.vision_sensor import VisionSensor

from scipy import misc

# import matplotlib.pyplot as plt

import pygame


# from PIL import Image

# w, h = 512, 512
# data = np.zeros((h, w, 3), dtype=np.uint8)
# data[0:256, 0:256] = [255, 0, 0] # red patch in upper left


from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError

from pyrep.robots.arms.arm import Arm


class Harvey(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Harvey', 7)


class pyrep_interface():


    def __init__(self, number_of_cameras, SCENE_FILE=None):

        self.type = "SIM"

        #setup VREP connection
        print ('initialising sim')
        if(SCENE_FILE == None):
            SCENE_FILE = join(dirname(abspath(__file__)), '../../vrep_scenes/PyRep_harvey.ttt')

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        self.agent = Harvey()

        # self.starting_joint_positions = agent.get_joint_positions()
        # pos, quat = agent.get_tip().get_position(), agent.get_tip().get_quaternion()
        self.nCameras = number_of_cameras
        self.camera = []
        self.colour_camera = []
        for index in range(number_of_cameras):
            self.camera.append(VisionSensor('Vision_sensor'+str(index)))
            self.colour_camera.append(VisionSensor('Vision_sensor_ref'+str(index)))

        # plt.ion()
        # plt.figure()
        # plt.show()
        self.resolution = self.colour_camera[0].get_resolution()
        self.image_width =  self.resolution[0]
        self.image_height =  self.resolution[1]

        hsv_thresholds_low = np.array([0, 120, 50])
        hsv_thresholds_high = np.array([12, 255, 255])

        self.lower_red = np.copy(hsv_thresholds_low)
        self.upper_red =  np.copy(hsv_thresholds_high)
        #
        self.lower_red2 =  np.copy(hsv_thresholds_low)
        self.lower_red2[0] = 180 - self.upper_red[0]
        self.upper_red2 = np.array([180,255,255])

        self.image_array = []
        for i in range(0,self.nCameras):
            self.image_array.append(0)

        self.camera_link = Dummy("camera_link")
        self.ref_camera = VisionSensor("Vision_sensor4")
        self.base_frame = Dummy("base_frame")
        self.ee_handle = Dummy("camera_link")

        self.capsicum = Shape("capsicum")
        self.leaf_0 = Shape("leaf_0")


        self.starting_joint_positions = np.array([0.0,-360,-270-45,-90,-135,-90,90]);

        #internal memory for pixel info
        self.pixel_sizes = self.init_list_of_objects(number_of_cameras, 0)
        self.pixel_sizes_filtered = self.init_list_of_objects(number_of_cameras, 0)
        self.blob_centres = self.init_list_of_objects(number_of_cameras, 0)


        #### for displaying Images
        pygame.init()
        self.first_call = True
        self.border=50
        self.screen = pygame.display.set_mode((3*self.image_width+(2*self.border), 3*self.image_height+(2*self.border)))
        pygame.display.set_caption("PyRep Images")
        done = False
        self.clock = pygame.time.Clock()

        self.screen.fill((255, 255, 255))

    def displayImage(self, image):

        surface = pygame.surfarray.make_surface(image)
        self.screen.blit(surface, (self.border, self.border))

        # Display and update frame counter
        # text = basicfont.render('Frame: ' + str(N), True, (255, 0, 0), (255, 255, 255))
        # self.screen.blit(text, (border,self.height+border))
        # N = N + 1
        pygame.display.flip()
        self.clock.tick(60)


    def __del__(self):
        self.pr.shutdown()


    def getRefImage(self):
        image = camera[4].capture_rgb()
        return image

    # def sin2d(self, x,y):
    # # """2-d sine function to plot"""
    #
    #     return np.sin(x) + np.cos(y)
    #
    # def getFrame(self):
    #
    # # Create data on first call only
    #     if self.first_call:
    #         self.first_call = False
    #         xx, yy = np.meshgrid(np.linspace(0,2*np.pi,self.image_height), np.linspace(0,2*np.pi,self.image_width))
    #         self.frame = self.sin2d(xx, yy)
    #         self.frame = 255*self.frame/self.frame.max()
    #
    #     # Just roll data for subsequent calls
    #     self.frame = np.roll(self.frame,(1,2),(0,1))
    #     return self.frame

    def getImage(self,camera_index):
        image = self.colour_camera[camera_index].capture_rgb()*255.0

        # img = Image.fromarray(image, 'RGB')
        # img.save('my.png')


        # plt.draw()
        # plt.show()
        # cv2.imshow("Images", image)
        # cv2.waitKey(1)

        return image

    def init_joints(self):
        return self.agent.set_joint_positions(np.deg2rad(self.starting_joint_positions))

    #Just used to initiliase starting pose, should be deprecated
    # def initialise_sim(self,ee_pose,frame=[]):
    #
    #     # pos, quat = agent.get_tip().get_position(), agent.get_tip().get_quaternion()
    #     joint_angles = agent.solve_ik(ee_pose[0:2], quaternion=ee_pose[3:7])
    #     agent.set_joint_target_positions(joint_angles)
    #     pr.step()


    def start_sim(self):
        print ("starting sim")
        self.pr.start()

    def stop_sim(self):
        print ("stopping sim")
        self.pr.stop()

    def disconnect_sim():
        print("Closing connection to vrep")
        self.pr.shutdown()

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

    def set_object_position(self,object_name,position=None,orientation=None,ref_name=None):
        res,handle = vrep.simxGetObjectHandle(self.clientID,object_name,vrep.simx_opmode_oneshot_wait)

        if ref_name is not None:
            ref_handle = -1
        else:
            res,ref_handle = vrep.simxGetObjectHandle(self.clientID,ref_name,vrep.simx_opmode_oneshot_wait)

        if position is not None:
            vrep.simxSetObjectPosition(self.clientID,handle,ref_handle,position,vrep.simx_opmode_oneshot_wait)
        if orientation is not None:
            # print orientation
            vrep.simxSetObjectOrientation(self.clientID,handle,ref_handle,orientation,vrep.simx_opmode_oneshot_wait)


    def set_joints_degrees(self,joint_values):
        self.agent.set_joint_positions(np.deg2rad(joint_values))

    def setCameraOffsets(self, radius, link_offset, angle, set_euler_angles=False):

        #offset from camera array to end effector
        link_offset = link_offset

        #if these are different makes a rectangular array not a square
        theta = np.deg2rad(angle)
        phi = np.deg2rad(angle)

        #use a grid to construct the camera array
        theta_grid=[math.pi/2 - theta,math.pi/2 - theta,math.pi/2 - theta,math.pi/2,math.pi/2,math.pi/2,math.pi/2 + theta, math.pi/2 + theta, math.pi/2 + theta]
        phi_grid=[phi,0,-1*phi,phi,0,-1*phi,phi,0,-1*phi]

        for i in range(0,9):
            # euler_angles = self.camera[j].get_orientation(self.ref_camera) #input is get orientation relative to

            if i == 3:
                z = 0
            else:
                z = -radius*math.sin(theta_grid[i])*math.cos(phi_grid[i]) + radius + link_offset

            x = radius*math.sin(theta_grid[i])*math.sin(phi_grid[i])
            y = radius*math.cos(theta_grid[i])

            # transform = sim.buildMatrix({x,y,z},euler_angles)
            # transform2 = sim.buildMatrix({0,0,0},{-phi_grid[i],theta_grid[i]-math.pi/2,0})
            # transform3 = sim.multiplyMatrices(transform,transform2)

            self.camera[i].set_position(np.array([x,y,z]),relative_to=self.camera_link)



    def getCurrentPose(self):
        emptyBuff = bytearray()
        pos = self.ee_handle.get_position(relative_to=self.base_frame_handle)
        orientation = self.ee_handle.get_orientation(relative_to=self.base_frame_handle)
        return position+orientation


    def getCameraPositions(self):
        inInts=[]
        inFloats=[]
        emptyBuff = bytearray()

        camera_positions = np.zeros([3,self.nCameras])
        camera_poses = np.zeros([6,self.nCameras])
        camera_orientations = np.zeros([3,self.nCameras])

        for i in range(0,self.nCameras):

            camera_position = self.camera[i].get_position(relative_to=self.ref_camera)
            camera_orientation = self.camera[i].get_orientation(relative_to=self.ref_camera)

            camera_positions[0:3,i] = camera_position
            camera_orientations[0:3,i] = camera_orientation
            camera_poses[0:6,i] = camera_position + camera_orientation


        return camera_positions,camera_orientations,camera_poses


    def getObjectiveFunctionValues(self):


        t2 = time.time()
        #print("Time to get objective function values = ", t2-t1)

        delta_matrix = np.zeros([3,3])
        size_matrix = np.zeros([3,3])
        distance_matrix = np.zeros([3,3])
        manip_matrix = np.zeros([3,3])

        # print retFloats
        pixel_size = []
        blob_centre = []
        manip = []
        segmentedImage_array = []
        objects = []


        for i in range(0,self.nCameras):
            self.image_array[i] = self.getImage(i)
            # object, image = self.detect_objects(self.image_array[i])
            # objects.append(object)
            # segmentedImage_array.append(image)
            #
            # if len(object) > 0:
            #     self.blob_centres[i] = (object['centre_x'], object['centre_y'])
            #     #divide by maximum pixel size
            #     if(self.normalise_pixels):
            #         self.pixel_sizes[i] = object['size']/float(self.image_width*self.image_height)
            #     else:
            #         self.pixel_sizes[i] = object['size']
            #     #keep a sliding window of sizes for filtering
            #     self.pixel_sizes_buffer[i].append(copy.deepcopy(self.pixel_sizes[i]))
            #     del self.pixel_sizes_buffer[i][0]
            #     self.pixel_sizes_filtered[i] = sum(self.pixel_sizes_buffer[i])/self.window_size

        row_1 = np.hstack((self.image_array[2], self.image_array[1], self.image_array[0]))
        row_2 = np.hstack((self.image_array[5], self.image_array[4], self.image_array[3]))
        row_3 = np.hstack((self.image_array[8], self.image_array[7], self.image_array[6]))
        image_matrix = np.vstack((row_1, row_2, row_3))

        # seg_row_1 = np.hstack((segmentedImage_array[2], segmentedImage_array[1], segmentedImage_array[0]))
        # seg_row_2 = np.hstack((segmentedImage_array[5], segmentedImage_array[4], segmentedImage_array[3]))
        # seg_row_3 = np.hstack((segmentedImage_array[8], segmentedImage_array[7], segmentedImage_array[6]))
        # seg_image_matrix = np.vstack((seg_row_1, seg_row_2, seg_row_3))

        self.displayImage(image_matrix)
        # cv2.imshow("Images", image_matrix)
        # cv2.imshow("Segmented Images", seg_image_matrix)

        # cv2.waitKey(1)


        # ret_images = copy.deepcopy(self.cv_image_array)

        # return self.pixel_sizes_filtered, self.blob_centres
        # return self.pixel_sizes_filtered,self.blob_centres,self.pixel_sizes,ret_images, objects



    def servoPoseEuler(self, delta_pose):
        # start_pos, start_euler = self.agent.get_tip().get_position(relative_to=self.ref_camera), self.get_tip().get_orientation(relative_to=self.ref_camera)

        #hack to set ik target relative to reference camera
        self.agent._ik_target.set_position(delta_pose[0:3], relative_to=self.ref_camera)
        self.agent._ik_target.set_position(delta_pose[3:6], relative_to=self.ref_camera)

        #once ik tarket set, grab pose relative to base frame
        pos_in_base_frame = self.agent._ik_target.get_position()
        euler_in_base_frame = self.agent._ik_target.get_position()

        try:
            #finds ik solution relative to base frame
            joint_angles = self.agent.solve_ik(pos_in_base_frame, euler=euler_in_base_frame)
        except IKError:
            print ('IK failed')
            return False

        self.agent.set_joint_target_positions(joint_angles)
        pr.step()
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
