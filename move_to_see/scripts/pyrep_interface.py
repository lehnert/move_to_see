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

# RTB Control Imports
import roboticstoolbox as rtb 
from spatialmath import SE3
from numpy import pi
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt
import os
import pickle

class Harvey(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Harvey', 7)


class pyrep_interface():


    def __init__(self, number_of_cameras, SCENE_FILE=None):

        self.type = "SIM"

        #setup VREP connection
        print ('initialising sim')
        if(SCENE_FILE == None):
            # SCENE_FILE = join(dirname(abspath(__file__)), '../../vrep_scenes/PyRep_harvey_cocked_back.ttt')
            # SCENE_FILE = join(dirname(abspath(__file__)), '../../vrep_scenes/PyRep_harvey_cocked_back_default.ttt')
            SCENE_FILE = join(dirname(abspath(__file__)), '../../vrep_scenes/PyRep_harvey1.ttt')

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        self.agent = Harvey()
        
        # self.agent.set_ik_group_properties(resolution_method='pseudo_inverse')

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

        self.max_contour_size = 500
        self.normalise_pixels = True
        # hsv_thresholds_low = np.array([0, 120, 50])
        # hsv_thresholds_high = np.array([12, 255, 255])

        hsv_thresholds_low = np.array([0, 0, 50])
        hsv_thresholds_high = np.array([30, 255, 255])



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


        # self.starting_joint_positions = np.array([0.0,-360,-270-45,-90,-135,-90,90]);
        # self.starting_joint_positions = np.array([0.0,-360,-270-45,-90,-135,-90,90]);
        # self.starting_joint_positions = np.array([0.0,0,25,-100,75,90,0]); # most recent
        self.starting_joint_positions = np.rad2deg(self.agent.get_joint_positions())

        #internal memory for pixel info
        self.pixel_sizes = self.init_list_of_objects(number_of_cameras, 0)
        self.pixel_sizes_filtered = self.init_list_of_objects(number_of_cameras, 0)
        self.blob_centres = self.init_list_of_objects(number_of_cameras, 0)

        #parameter and memory for moving avg filtering
        self.window_size = 5
        self.pixel_sizes_buffer = self.init_list_of_objects(number_of_cameras,self.window_size)

        #### for displaying Images
        pygame.init()
        self.first_call = True
        self.border=50
        self.screen = pygame.display.set_mode((3*self.image_width+(2*self.border), 3*self.image_height+(2*self.border)))
        pygame.display.set_caption("PyRep Images")
        done = False
        self.clock = pygame.time.Clock()

        self.screen.fill((255, 255, 255))
        
        
        ## Harvey Model for RTB Control
        self.lift_link = False # True: enables lift link (7dof); False: disables lift link (6dof, regular UR5)
        # base transform
        base_CS = self.agent.get_object('base_frame') # base frame for relative positions
        b_pos_CS = base_CS.get_position()
        b_rpy_CS = base_CS.get_orientation()

        b_SE3_CS = SE3().Rx(b_rpy_CS[0])
        b_SE3_CS = b_SE3_CS.Ry(b_rpy_CS[1])
        b_SE3_CS = b_SE3_CS.Rz(b_rpy_CS[2], t=b_pos_CS)
        # b_SE3_CS = SE3().Rx(b_rpy_CS[0]).Ry(b_rpy_CS[1]).Rz(b_rpy_CS[2], t=b_pos_CS)

        # tool transform
        tip_CS = self.agent.get_object('Harvey_tip') # base frame for relative positions
        t_pos_CS = tip_CS.get_position(relative_to=self.agent.get_object('Harvey_joint7'))
        t_rpy_CS = tip_CS.get_orientation(relative_to=self.agent.get_object('Harvey_joint7'))

        t_SE3_CS = SE3().Rx(t_rpy_CS[0])
        t_SE3_CS = t_SE3_CS.Ry(t_rpy_CS[1])
        t_SE3_CS = t_SE3_CS.Rz(t_rpy_CS[2], t=t_pos_CS)
        lift_pos_CS = self.agent.get_object('Harvey_joint1').get_position(relative_to=base_CS)
        
        a     = [ 0,         0,         -0.425,     -0.39225,    0,        0,       0] # [m]
        d     = [ 0,         0.089159,   0,          0,          0.10915,  0.09465, 0.0823] # [m]
        al    = [-pi/2,      pi/2,       0,          0,          pi/2,    -pi/2,    0] # [rad] '''pi/2'''
        th    = np.zeros(7) # [rad]
        ofs    = [lift_pos_CS[2], 0,     -pi/2,       0,         -pi/2,     0,       -pi/2] # offsets

        # m[0] might not be right
        m  	  = [0, 3.7, 8.393, 2.33, 1.219, 1.219, 0.1879] # mass (kg)
        g     = [0, 0, 9.81] # gravity acting in Z
        links = []
        if self.lift_link: links.append(rtb.PrismaticDH(theta=th[0], a=a[0], alpha=al[0], m=m[0], offset=ofs[0], qlim=[-0.5, 0.5]))
        [links.append(rtb.RevoluteDH(d=d[l], a=a[l], alpha=al[l], m=m[l], offset=ofs[l])) for l in range(1,7)]
        self.H_rtb = rtb.DHRobot(links=links, name='UR5_harvey', manufacturer='Universal Robotics',
                    base=b_SE3_CS, tool=t_SE3_CS)#b_mat)
        
        self.H_rtb.q = self.agent.get_joint_positions()[7-len(self.H_rtb.links):]
        
        self.env = PyPlot()
        self.env.launch('3D Move to See')
        self.env.add(self.H_rtb)
        ## end RTB control model - see servoRTB function for use of the model

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
        image = self.colour_camera[4].capture_rgb()
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

    def step_sim(self):
        self.pr.step()

    def disconnect_sim(self):
        print("Closing connection to sim")
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

    def setCameraOffsets(self, xy_offset, z_offset, link_offset, set_euler_angles=False):

        #offset from camera array to end effector
        link_offset = link_offset

        row00 = [-xy_offset, xy_offset, z_offset] #0
        row01 = [0, xy_offset, z_offset/2] #1
        row02 = [xy_offset, xy_offset, z_offset] #2
        row10 =[-xy_offset, 0, z_offset/2] #3
        row11 = [0,0,0] #4
        row12 = [xy_offset, 0, z_offset/2] #5
        row20 = [-xy_offset, -xy_offset, z_offset] #6
        row21 = [0, -xy_offset, z_offset/2] #7
        row22 = [xy_offset, -xy_offset, z_offset] #8
        grid = np.array([[row00, row01, row02],[row10, row11, row12],[row20, row21, row22]])

        index = 0
        for i in range(0,3):
            for j in range(0,3):

                x = grid[i][j][0]
                y = grid[i][j][1]
                z = grid[i][j][2]

                self.camera[index].set_position(np.array([x,y,z]),relative_to=self.camera_link)
                self.camera[index].set_orientation(np.array([0,0,0]),relative_to=self.camera_link)
                index = index + 1


    def getCurrentPose(self):
        emptyBuff = bytearray()
        pos = self.ee_handle.get_position(relative_to=self.base_frame)
        orientation = self.ee_handle.get_orientation(relative_to=self.base_frame)
        return pos+orientation


    def getCameraPositions(self):

        camera_positions = np.zeros([3,self.nCameras])
        camera_poses = np.zeros([6,self.nCameras])
        camera_orientations = np.zeros([3,self.nCameras])

        for i in range(0,self.nCameras):

            camera_position = self.camera[i].get_position(relative_to=self.ref_camera)
            camera_orientation = self.camera[i].get_orientation(relative_to=self.ref_camera)
            print(camera_position)
            print(camera_orientation)
            camera_positions[0:3,i] = camera_position
            camera_orientations[0:3,i] = camera_orientation
            camera_poses[0:3,i] = camera_position
            camera_poses[3:6,i] = camera_orientation


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
            object, image = self.detect_objects(self.image_array[i])

            objects.append(object)
            segmentedImage_array.append(image)
            #
            if len(object) > 0:
                self.blob_centres[i] = (object['centre_x'], object['centre_y'])
                #divide by maximum pixel size
                if(self.normalise_pixels):
                    self.pixel_sizes[i] = object['size']/float(self.image_width*self.image_height)
                else:
                    self.pixel_sizes[i] = object['size']
                #keep a sliding window of sizes for filtering
                self.pixel_sizes_buffer[i].append(self.pixel_sizes[i])
                del self.pixel_sizes_buffer[i][0]
                self.pixel_sizes_filtered[i] = sum(self.pixel_sizes_buffer[i])/self.window_size

        row_1 = np.hstack((self.image_array[2], self.image_array[1], self.image_array[0]))
        row_2 = np.hstack((self.image_array[5], self.image_array[4], self.image_array[3]))
        row_3 = np.hstack((self.image_array[8], self.image_array[7], self.image_array[6]))
        image_matrix = np.vstack((row_1, row_2, row_3))

        seg_row_1 = np.hstack((segmentedImage_array[2], segmentedImage_array[1], segmentedImage_array[0]))
        seg_row_2 = np.hstack((segmentedImage_array[5], segmentedImage_array[4], segmentedImage_array[3]))
        seg_row_3 = np.hstack((segmentedImage_array[8], segmentedImage_array[7], segmentedImage_array[6]))
        seg_image_matrix = np.vstack((seg_row_1, seg_row_2, seg_row_3))

        self.displayImage(seg_image_matrix)
        # cv2.imshow("Images", image_matrix)
        # cv2.imshow("Segmented Images", seg_image_matrix)

        # cv2.waitKey(1)


        # ret_images = copy.deepcopy(self.cv_image_array)

        # return self.pixel_sizes_filtered, self.blob_centres
        return self.pixel_sizes_filtered,self.blob_centres,self.pixel_sizes,self.image_array, objects



    def servoPoseEuler(self, delta_pose, plot=True):
        # start_pos, start_euler = self.agent.get_tip().get_position(relative_to=self.ref_camera), self.get_tip().get_orientation(relative_to=self.ref_camera)
        print("Servoing to Delta Pose")
        print(delta_pose)
        #hack to set ik target relative to reference camera
        self.agent._ik_target.set_position(delta_pose[0:3], relative_to=self.ref_camera)
        self.agent._ik_target.set_orientation(delta_pose[3:6], relative_to=self.ref_camera)

        #once ik tarket set, grab pose relative to base frame
        pos_in_base_frame = self.agent._ik_target.get_position()
        euler_in_base_frame = self.agent._ik_target.get_orientation()
        
        ## PLOTTING - end effector position, joint velocities, manipulability
        if plot:
            try: self.dt
            except AttributeError: 
                self.dt = self.pr.get_simulation_timestep()
            
                self.t = [0]
                self.q = self.agent.get_joint_positions()
                self.m = [self.H_rtb.manipulability(q=self.H_rtb.q)] # initial manipulability
                self.qd = self.agent.get_joint_velocities() # initial joint velocities
                self.X = [self.agent.get_tip().get_position(relative_to=self.base_frame)] # initial position
                
            self.q = np.vstack((self.q, self.agent.get_joint_positions()))
            self.qd = np.vstack((self.qd, self.agent.get_joint_velocities())) #self.agent.get_joint_velocities()[7-len(self.H_rtb.links):]))
            self.t.append(self.t[-1]+self.dt) # expand time vector
            self.m = np.vstack((self.m, self.H_rtb.manipulability(q=self.agent.get_joint_positions())))
            self.X = np.vstack((self.X, self.agent.get_tip().get_position(relative_to=self.base_frame))) # defaults 2D array
            
            mqdflag = True
            pflag = False
            tflag = True
            ptype = 'Singularity_6.5dof' # type of test being run
            plotname = 'plots/CS_'+ptype # partial only
            # Initialisation
            try: self.firstrunplot
            except AttributeError:
                self.firstrunplot = True
                
                self.Xlabels = ["X (m)", "Y (m)", "Z (m)"]
                self.jlabels = ["Joint {}".format(i) for i in np.arange(1,7+1)]  
                
                # manipulability, joint velocity init
                if mqdflag:
                    self.mqdfig, self.mqdax = plt.subplots(2)
                    self.mqdfig.suptitle('Manipulability and Joint Velocity')
                    self.mqdax[0].set_title('Manipulability and Joint Velocity')
                    self.mqdax[1].sharex(self.mqdax[0]) # share axis

                # XYZ position init
                if pflag:
                    self.pfig, self.pax = plt.subplots(3)
                    self.pfig.suptitle('End Effector Position')
                    self.pax[0].set_title('End Effector Position')
                
                # trajectory init
                if tflag:
                    self.tfig = plt.figure()
                    self.tax = self.tfig.add_subplot(projection='3d')                
                    self.tfig.suptitle('Trajectory')
                    self.tax.set_title('Trajectory')
                    self.fpos = self.capsicum.get_position(relative_to=self.base_frame)
            
            
            # manipulability and qd plots update       
            if mqdflag:
                [self.mqdax[i].cla() for i in range(2)] # clear axes
                self.mqdax[0].plot(self.t, self.m, label="Manipulability", linewidth=1.2) 
                self.mqdax[1].plot(self.t, self.qd, label=self.jlabels, linewidth=1.2)
                
                self.mqdax[0].set(ylabel='Manipulability')
                self.mqdax[1].set(xlabel='Time (s)', ylabel='Joint Velocity')
                self.mqdax[1].legend(loc='upper right')
                self.mqdfig.savefig(plotname+'_Manip.png', bbox_inches='tight', pad_inches=0.05)

            # displacement plots update
            if pflag:
                [self.pax[i].cla() for i in range(3)] # clear axis
                [self.pax[i].plot(self.t, self.X[:,i], color='red', label='Actual', linewidth=1.5) for i in range(3)] # position, joint labels
                            
                [self.pax[i].set(ylabel=self.Xlabels[i]) for i in range(3)]
                self.pax[2].set(xlabel='Time (s)')
                # self.pax.legend(loc='upper right')
                self.pfig.savefig(plotname+'_Pos.png', bbox_inches='tight', pad_inches=0.05)
            
            # trajectory plot update
            if tflag:
                self.tax.cla()
                self.tax.plot(self.X[:,0], self.X[:,1], self.X[:,2], color='red', label='Trajectory', linewidth=1.5)
                self.tax.scatter(self.fpos[0], self.fpos[1], self.fpos[2], s=100, label='Target Capsicum')
                self.tax.set_xlabel(self.Xlabels[0])
                self.tax.set_ylabel(self.Xlabels[1])
                self.tax.set_zlabel(self.Xlabels[2])
                self.tax.legend(loc='upper right')
                self.tfig.savefig(plotname+'_Traj.png', bbox_inches='tight', pad_inches=0.05)
            plt.show()

        try:
            #finds ik solution relative to base frame
            joint_angles = self.agent.solve_ik(pos_in_base_frame, euler=euler_in_base_frame)
        except IKError:
            print ('IK failed')
            return False

        self.agent.set_joint_target_positions(joint_angles)
        # for i in range(0,20):
        #     self.pr.step()

        return True

    def detect_objects(self, image):
           # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        hsv = cv2.GaussianBlur(hsv,(5,5),0)

        # Threshold the HSV image to get only blue colors
        mask1 = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1,mask2)

        erode = cv2.erode(mask,np.ones((5,5)))
        mask = cv2.dilate(erode,np.ones((5,5)))

        segmentedImage = cv2.bitwise_and(image,image, mask=mask)
        # segmentedImage = hsv

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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
  
    
    def servoRTB(self, delta_pose, ctrl='DLS', damping=1e-1, plot=True, save=False):
        try: self.dt
        except AttributeError: 
            self.dt = self.pr.get_simulation_timestep()
            
            if plot or save:
                self.t = [0]
                self.m = [self.H_rtb.manipulability(q=self.H_rtb.q, J=self.H_rtb.jacob0(self.H_rtb.q))] # initial manipulability
                self.qd = self.agent.get_joint_velocities()[7-len(self.H_rtb.links):]#[self.H_rtb.qd] # initial joint velocities
                # self.X = [self.H_rtb.fkine(self.H_rtb.q).t] # initial position
                self.X = [self.agent.get_tip().get_position(relative_to=self.base_frame)] # initial position
                # self.X = [self.H_rtb.tool.t]
            
            print('Timestep is {}'.format(self.dt))
        # replace H_CS with self.agent
        # see line 540 RTBControl
        print("Servoing to Delta Pose")
        print(delta_pose)
        delta_pose = delta_pose/self.dt # to make it a velocity in m/s # slightly slowed down
        print("Modified Delta Pose:")
        print(delta_pose)
        # the actual movement function
        self.H_rtb.q = self.agent.get_joint_positions()[7-len(self.H_rtb.links):]
        
        start = time.time()
        J_rtb = self.H_rtb.jacobe(self.H_rtb.q) # jacobian rel to camera/EE
        JT_rtb = np.transpose(J_rtb)
                
        if ctrl == 'JT': # JT Control (Kv = 30 is good)
            # alpha = dot(e, J J^T e) / dot(J J^T e, J J^T e)
            al = np.dot(delta_pose, J_rtb @ JT_rtb @ delta_pose) / np.dot(J_rtb @ JT_rtb @ delta_pose, J_rtb @ JT_rtb @ delta_pose)
            # al = 0.5
            self.H_rtb.qd = al*JT_rtb @ delta_pose
            print("Alpha = %f" % al)
            
        # elif ctrl == 'Corke':
        #     B = 0.5 # joint damping coefficient
        #     T = SE3(delta_pose[0], delta_pose[1], delta_pose[2])
        #     T = T.RPY(delta_pose[3], delta_pose[4], delta_pose[5])
        #     dq = self.H_rtb.ikine_LM(T, q0=self.H_rtb.q)
        #     fk = self.H_rtb.fkine(dq.q)
        #     dp = np.hstack([fk.t, fk.rpy()])
        #     self.H_rtb.qd = 1/B * JT_rtb @ dp
            
        elif ctrl == 'PINV': self.H_rtb.qd = np.linalg.pinv(J_rtb) @ delta_pose # PINV control
        
        elif ctrl == 'PINVNull': # PINV nullspace control - not very good
            J_pinv =  np.linalg.pinv(J_rtb)
            p = 1e-2
            phi = np.ones(len(self.H_rtb.links))*p
            I = len(self.H_rtb.links)            
            self.H_rtb.qd = J_pinv @ delta_pose + (np.eye(I) - J_pinv @ J_rtb) @ phi 
            
        elif ctrl == 'DLS': # damped pseudoinverse - use this unless you're specifically trying to improve the others
            lda0 = damping # lambda_0: max damping factor
            wt = 0.01 # threshold manipulability (determined experimentally)
            w = self.H_rtb.manipulability(q=self.H_rtb.q)
            
            if w < wt: lda = lda0*(1-(w/wt)) # manip under threshold: add scaling damping factor
            else: lda = 0 # manipulability over threshold, no damping needed
            
            # lda = lda0 # testing - constant rather than adaptive damping
            J_dpinv = JT_rtb @ np.linalg.inv(J_rtb @ JT_rtb + lda**2 * np.eye(6))
            self.H_rtb.qd = J_dpinv @ delta_pose
            
        else: raise Exception("Invalid Control String - must be one of ['JT | 'PINV' | 'PINVNull' | 'DLS'] (use DLS if you are unsure)")
        
        ignore_lift = False # set this if you want the lift link included in calcs but never moving
        if ignore_lift and self.lift_link: self.H_rtb.qd[0] = 0
        if self.lift_link: self.agent.set_joint_target_velocities(self.H_rtb.qd) # set normally
        else: self.agent.set_joint_target_velocities(np.hstack([0, self.H_rtb.qd])) # set lift link velocity to 0
                
        ## DATA - track position, velocity, manipulability over time
        if plot or save:
            self.t.append(self.t[-1]+self.dt) # expand time vector
            self.m = np.vstack((self.m, self.H_rtb.manipulability(q=self.H_rtb.q)))
            # self.X = np.vstack((self.X, self.H_rtb.fkine(self.H_rtb.q).t)) # defaults 2D array
            self.X = np.vstack((self.X, self.agent.get_tip().get_position(relative_to=self.base_frame))) # defaults 2D array
            # self.X = np.vstack((self.X, self.H_rtb.tool.t)) # defaults 2D array
            self.qd = np.vstack((self.qd, self.H_rtb.qd)) #self.agent.get_joint_velocities()[7-len(self.H_rtb.links):]))
            
        ## FILE IO - save above data in a pickle file in the pickles directory
        if save: 
            ext = ".pkl"
            contents = os.listdir(os.getcwd()+'/pickles') # get file list
            files = [file for file in contents if file.endswith(ext)] # only get `.ext` files
            if not files: curr = '1' # init file number (shouldn't trigger)
            else: 
                fnames = ['.'.join(file.split('.')[:-1]) for file in files] # remove extensions
                nums = [int('_'.join(x.split('_')[len(x.split('_'))-1:])) for x in fnames] # get the digits at the end of the filenames
                nums.sort()
                curr = str(nums[-1]) # get current filenum (incremented in test_move_to_see_pyrep_singularity.py)
            pname = 'pickles/test_' + curr + '.pkl' 
            data = {
                "t": self.t, 
                "m": self.m,
                "X": self.X,
                "qd": self.qd
                }
            pickle.dump(data, open(pname, 'wb'))
        
        ## PLOTTING - end effector position, joint velocities, manipulability
        if plot:
            mqdflag = True
            pflag = False
            tflag = True
            ptype = 'Singularity' # type of test being run
            dof = str(len(self.H_rtb.links))+'dof'
            if ignore_lift and self.lift_link: dof = '6.5dof'
            plotname = 'plots/'+ctrl+'_'+ptype+'_'+dof # partial only
            if ctrl == 'DLS': plotname = plotname + '_λ0=' + str(damping)
            # Initialisation
            try: self.firstrunplot
            except AttributeError:
                self.firstrunplot = True
                
                self.Xlabels = ["X (m)", "Y (m)", "Z (m)"]
                self.jlabels = ["Joint {}".format(i) for i in np.arange(1,len(self.H_rtb.links)+1)]  
                
                # manipulability, joint velocity init
                if mqdflag:
                    self.mqdfig, self.mqdax = plt.subplots(2)
                    self.mqdfig.suptitle('Manipulability and Joint Velocity')
                    self.mqdax[0].set_title('Manipulability and Joint Velocity')
                    self.mqdax[1].sharex(self.mqdax[0]) # share axis

                # XYZ position init
                if pflag:
                    self.pfig, self.pax = plt.subplots(3)
                    self.pfig.suptitle('End Effector Position')
                    self.pax[0].set_title('End Effector Position')
                
                # trajectory init
                if tflag:
                    self.tfig = plt.figure()
                    self.tax = self.tfig.add_subplot(projection='3d')                
                    self.tfig.suptitle('Trajectory')
                    self.tax.set_title('Trajectory')
                    self.fpos = self.capsicum.get_position(relative_to=self.base_frame)
            
            
            # manipulability and qd plots update       
            if mqdflag:
                [self.mqdax[i].cla() for i in range(2)] # clear axes
                self.mqdax[0].plot(self.t, self.m, label="Manipulability", linewidth=1.2) 
                self.mqdax[1].plot(self.t, self.qd, label=self.jlabels, linewidth=1.2)
                
                self.mqdax[0].set(ylabel='Manipulability')
                self.mqdax[1].set(xlabel='Time (s)', ylabel='Joint Velocity')
                self.mqdax[1].legend(loc='upper right')
                self.mqdfig.savefig(plotname+'_Manip.png', bbox_inches='tight', pad_inches=0.05)

            # displacement plots update
            if pflag:
                [self.pax[i].cla() for i in range(3)] # clear axis
                # [self.pax[i].plot(self.t, self.X[:,i], label=self.Xlabels[i]) for i in range(3)] # position, joint labels
                [self.pax[i].plot(self.t, self.X[:,i], color='red', label='Actual', linewidth=1.5) for i in range(3)] # position, joint labels
                # [self.pax[i].plot(self.t, self.Xstar[:,i], color='blue', linestyle=(0, (5, 10)), label='Ideal', linewidth=1.5) for i in range(3)]
            
                [self.pax[i].set(ylabel=self.Xlabels[i]) for i in range(3)]
                self.pax[2].set(xlabel='Time (s)')
                # self.pax.legend(loc='upper right')
                self.pfig.savefig(plotname+'_Pos.png', bbox_inches='tight', pad_inches=0.05)
            
            # trajectory plot update
            if tflag:
                self.tax.cla()
                self.tax.plot(self.X[:,0], self.X[:,1], self.X[:,2], color='red', label='Trajectory', linewidth=1.5)
                self.tax.scatter(self.fpos[0], self.fpos[1], self.fpos[2], s=100, label='Target Capsicum')
                self.tax.set_xlabel(self.Xlabels[0])
                self.tax.set_ylabel(self.Xlabels[1])
                self.tax.set_zlabel(self.Xlabels[2])
                self.tax.legend(loc='upper right')
                self.tfig.savefig(plotname+'_Traj.png', bbox_inches='tight', pad_inches=0.05)
            plt.show()