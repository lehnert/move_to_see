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
import scipy.io
import pickle

import torch

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2

from ros_move_to_see import move_to_see

if __name__=="__main__":

    rospy.init_node("test")

    mvs = move_to_see(9,"ROS",step_size=0.001, size_weight=1.0, manip_weight=0.0,end_tolerance=0.75,max_pixel=0.4,max_count=100)
    mvs.initCameraPosition()

    # print "Setting arm to initial position"
    # if not mvs.interface.movetoNamedPose("harvey_arm","move_to_see_start_v4", 0.4):
    #     print "failed to reset robot arm"
    #     exit()

    # prepare CNN model

    device_type = "cuda:1" if torch.cuda.is_available() else "cpu"

    device = torch.device(device_type)     # rsetup CNN to run on GPU2 if available
    MTS_CNN_model = torch.load('/home/chris/move_to_see_data/CNN_models/MTSmodel4_ep50_bs64_j0_lr00001_lrd01_t0421_v1751.pt', map_location=device_type) # load finetuned CNN (targets: DirGrad x, y, z)
    MTS_CNN_model.to(device)                                                    # load CNN to processing device (GPU)

    # save location of trail data

    data = mvs.execute(move_robot=True,CNN_model=MTS_CNN_model,device=device)

    print "Setting arm to initial position"
    if not mvs.interface.movetoNamedPose("harvey_arm","move_to_see_start_v4", 0.4):
        print "failed to reset robot arm"
        exit()

    path = "/home/chris/move_to_see_data/CNN_trials/"

    file_name = path + "/cnn_data_capture_" + time.strftime("%Y_%m_%d-%H_%M_%S")

    scipy.io.savemat(file_name+".mat", data)

    fileObject = open(file_name+".pickle",'wb')
    pickle.dump(data,fileObject)
    fileObject.close()

    print "Finished capturing data"
