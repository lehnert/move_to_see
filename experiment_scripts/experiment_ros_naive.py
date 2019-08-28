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

from ros_naive_move_to_see_interfaces import move_to_see
import time
import numpy as np
import math
import scipy.io
import matplotlib.pyplot as plt
import rospy
import sys

import pickle

def run(trial_number, angle):


    mvs = move_to_see(9,"ROS",step_size=0.001, size_weight=1.0, manip_weight=0.0,end_tolerance=0.75,max_pixel=0.3)

    #mvs.initCameraPosition()


    print "Setting arm to initial position"
    if not mvs.interface.movetoNamedPose("harvey_arm","move_to_see_start_v5", 0.4):
        print "failed to reset robot arm"
        return

    #start_image = mvs.interface.getRefCameraImage()

    data = mvs.executeNaive()

    #end_image = mvs.interface.getRefCameraImage()

    data['weights'] = [1.0,0.0]
    #data['start_image'] = start_image
    #data['end_image'] = end_image
    data['trial'] = trial_number
    data['angle'] = angle



    path = "/home/harvey/harvey_ws/src/harvey/harvey_move_to_see/scripts/real_robot_experiment/"

    file_name = path + "real_robot_naive"+ "_trial_"+str(trial_number) + "_" +str(angle) + "_" + time.strftime("%Y_%m_%d-%H_%M_%S")


    scipy.io.savemat(file_name+".mat", data)

    fileObject = open(file_name+".pickle",'wb')
    pickle.dump(data,fileObject)
    fileObject.close()

    print ('Trial ended')

if __name__=="__main__":



    rospy.init_node("harvey_move_to_see")

    trial = sys.argv[1]
    angle = sys.argv[2]

    print "Running experiment trial: ", trial, ", angle ", angle

    t1 = time.time()
    run(trial,angle)
    t2 = time.time()

    print "Total Time to run Experiment: ", t2-t1
    #cProfile.run('move_to_see()')
