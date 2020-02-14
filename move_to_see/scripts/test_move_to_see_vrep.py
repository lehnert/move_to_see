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

import scipy.io
import pickle

import cv2

from move_to_see import move_to_see

if __name__=="__main__":

    # capsicum position with respect to V-REP world frame
    cx = 0.4                
    cy = 0.6
    cz = 0.7
    

    # 3DMTS camera array parameters
    radius = 0.07               # size of 3DMTS camera array
    camera_angle = 30           # arrangement of cameras in array
    link_offset = 0.0           # camera array offset from end effector
    set_euler_angles = False    # ???

    # set-up fruit position with random offset (represents noise in starting position of end effector)
    offset_x = np.random.uniform(-0.05,0.05)
    offset_y = np.random.uniform(-0.05,0.05)
    offset_z = np.random.uniform(-0.05,0.05)
    fruit_pos = [cx, cy, cz]
    fruit_pos_offset = [cx + offset_x,cy + offset_y,cz + offset_z]  # comment out if offset not required
    
    # set-up occluding leaf in random position infront of fruit (with same random offset)
    occlusion_y = np.random.uniform(-0.06,0.06)
    occlusion_z = np.random.uniform(-0.08,0.08)
    occlusion_angle = np.random.uniform(-math.pi/2,math.pi/2)
    leaf_pos = [cx - 0.1,cy + occlusion_y,cz + occlusion_z]
    leaf_pos_offset = [cx + offset_x - 0.1,cy + offset_y + occlusion_y,cz + offset_z + occlusion_z] # comment out if offset not required
    

    mts = move_to_see(9,"VREP", step_size=0.001,            # MTS run conditions
                    size_weight=1.0, manip_weight=0.0,    # MTS objective funtion weights
                    end_tolerance=1.5,max_pixel=0.4)      # MTS termination conditions mts.initCameraPosition()
    
    mts.interface.start_sim()       # start sim - ensure V-REP scene is loaded first
    time.sleep(3)                   # allow time for V-REP to initiate sim

    # initialise positision of objects in scene
    mts.interface.init_joints()
    
    mts.interface.set_object_position('capsicum',fruit_pos_offset,[],'base_frame')
    mts.interface.set_object_position('leaf_0',leaf_pos_offset,[occlusion_angle,0.0,0.0],'base_frame')
    mts.setCameraPosition(radius,link_offset,camera_angle,set_euler_angles)

    # print "Setting arm to initial position"
    # if not mts.interface.movetoNamedPose("harvey_arm","move_to_see_start_v4", 0.4):
    #     print "failed to reset robot arm"
    #     exit()

    data = mts.execute(move_robot=True)

    # print "Setting arm to initial position"
    # if not mts.interface.movetoNamedPose("harvey_arm","move_to_see_start_v4", 0.4):
    #     print "failed to reset robot arm"
    #     exit()

    # path = "/home/chris/move_to_see_data"

    # file_name = path + "/cnn_data_capture_" + time.strftime("%Y_%m_%d-%H_%M_%S")

    # scipy.io.savemat(file_name+".mat", data)

    # fileObject = open(file_name+".pickle",'wb') 
    # pickle.dump(data,fileObject)
    # fileObject.close()

    print ("Finished capturing data")
