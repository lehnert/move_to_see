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



try:
    import vrep
except:
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import move_to_see
import time
import cv2 as cv
import array
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import math
import cProfile
from PIL import Image as I
import pickle
#import pylab as plb


def set_object_position(clientID,object_name,position,orientation):
    res,handle = vrep.simxGetObjectHandle(clientID,object_name,vrep.simx_opmode_oneshot_wait)

    if not position == []:
        vrep.simxSetObjectPosition(clientID,handle,-1,position,vrep.simx_opmode_oneshot_wait)
    if not orientation == []:
        # print orientation
        vrep.simxSetObjectOrientation(clientID,handle,-1,orientation,vrep.simx_opmode_oneshot_wait)

def run():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    nCameras = 9
    res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)

    if clientID!=-1:
        print ('Connected to remote API server')
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
        print('Started')

        yzSteps = 25
        xSteps = 25

        dist = 0.2

        cx = -0.4
        cy = 0.0
        cz = 0.5

        x_range = np.arange(cx-dist,cx+dist,(abs((cx-dist)-(cx+dist)))/xSteps)  
        y_range = np.arange(cy-dist,cy+dist,(abs((cy-dist)-(cy+dist)))/yzSteps)
        z_range = np.arange(cz-dist,cz+dist,(abs((cz-dist)-(cz+dist)))/yzSteps)
        # print x_range

        cost_map = np.zeros([yzSteps,yzSteps,xSteps])

        Y, Z, X = np.meshgrid(y_range, z_range, x_range)

        print Y.shape
        print Z.shape
        print X.shape

        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait) # ensure simulation has been restarted
        time.sleep(2)
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait) # Start the simulation to V-REP

        set_object_position(clientID,'capsicum',[-0.85,0.0,0.5],[])
        set_object_position(clientID,'leaf_0',[-0.75,-0.1,0.5],[-math.pi/4,0.0,0.0])
        set_object_position(clientID,'leaf_1',[-0.75,0.1,0.5],[math.pi/4,0.0,0.0])

        for x in range(X.shape[0]):
            for i in range(Y.shape[0]):
                for j in range(Y.shape[1]):
                        move_to_see.setPose([X[i,j,x],Y[i,j,x],Z[i,j,x],-0.707,0.0,0.707,0.0],clientID)

                        pixel_size, blob_centre, manip = move_to_see.getRawCostValues(clientID)

                        cost_map[i,j,x] = 1 - pixel_size[4]

        pickle.dump(cost_map, open( "cost_map.pickle", "wb" ))

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        #
        # surf = ax.plot_surface(Y, Z, cost_map, cmap=cm.coolwarm,
        #                linewidth=0, antialiased=False)
        #
        # fig.colorbar(surf, shrink=0.5, aspect=5)
        #
        # plt.show()

        # Now send some data to V-REP in a non-blocking fashion:
        vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)

        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')

    # cv.destroyAllWindows()
    print ('Program ended')

if __name__=="__main__":
    run()
    #cProfile.run('move_to_see()')
