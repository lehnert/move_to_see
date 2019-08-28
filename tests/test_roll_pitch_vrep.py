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



from move_to_see_interfaces import move_to_see
import time
import numpy as np
import math
import matplotlib.pyplot as plt

def run():
    # print ('Program started')
    # vrep.simxFinish(-1) # just in case, close all opened connections
    # clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    # vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait) # ensure simulation has been restarted
    # time.sleep(2)
    # nCameras = 9
    # res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)
    

    # if clientID!=-1:
    #     print ('Connected to remote API server')
    #     # Now try to retrieve data in a blocking fashion (i.e. a service call):
    #     res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    #     print('Started')

    nSteps = 2
    occlusion_steps = 2
    orientation_steps = 2
    radius_steps = 2
    link_offset_steps = 2
    angle_steps = 2

    dist = 0.01

    cx = -0.6
    cy = 0.0
    cz = 0.6
    
    camera_angle = 30
    link_offset = 0.0

    x_range = np.linspace(cx-dist,cx+dist,nSteps)
    y_range = np.linspace(cy-dist,cy+dist,nSteps)
    z_range = np.linspace(cz-dist,cz+dist,nSteps)

    leaf_range = np.linspace(0.075,0.1,occlusion_steps)
    orientation_range = np.linspace(0.0,math.pi/3,orientation_steps)

    radius_range = np.linspace(0.1,0.15,radius_steps)
    #link_offset_range = np.linspace(0.01,0.05,link_offset_steps)
    #camera_angle_range = np.linspace(10,25,angle_steps)
    
    plt.clf()
    fig = plt.figure(2)
        
    plt.ion()
    plt.show()
    plt.hold(True)

    # print x_range

    for x in x_range:
        for y in y_range:
            for z in z_range:
                for occlusion in leaf_range:
                    for radius in radius_range:
                        for angle in orientation_range:
                            #for link_offset in link_offset_range:
                                #for camera_angle in camera_angle_range:

                            mvs = move_to_see(9,0.0025,"VREP", end_tolerance=1e-2,max_pixel=0.4)
                            
                            
                            mvs.interface.initialise_sim(np.array([-0.3,0.0,0.6]))
    
                            mvs.interface.set_object_position('capsicum',[x,y,z],[])
                            mvs.interface.set_object_position('leaf_0',[x+0.1,y-occlusion,z],[-angle,0.0,0.0])
                            mvs.interface.set_object_position('leaf_1',[x+0.1,y+occlusion,z],[angle,0.0,0.0])
    
                            set_euler_angles = False
                            mvs.interface.setCameraOffsets(radius,link_offset,camera_angle,set_euler_angles)
    
                            count_data, plot_data, plot_data2 = mvs.execute()
    
                            del mvs                               
                             
                            plt.figure(2)
                            plt.hold(True)
                            #plt.subplot(211)
                            plt.plot(count_data,plot_data,'r')
                    
                            #plt.subplot(212)
                            plt.plot(count_data,plot_data2,'b')
                            
                            fig.canvas.draw()
                            fig.canvas.flush_events()
    
                            #mvs.interface.stop_sim()

    # cv.destroyAllWindows()
    print ('Program ended')

if __name__=="__main__":
    run()
    #cProfile.run('move_to_see()')
