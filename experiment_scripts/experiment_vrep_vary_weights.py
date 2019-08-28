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

from move_to_see_interfaces import move_to_see
import time
import numpy as np
import math
import scipy.io
import matplotlib.pyplot as plt

import pickle

def run():


    nSteps = 3
    occlusion_steps = 2
    orientation_steps = 3
    radius_steps = 3
    noise_steps = 2
    #link_offset_steps = 1
    #angle_steps = 1
    
    #nSteps = 1
    #occlusion_steps = 1
    #orientation_steps = 1
    #radius_steps = 1
    #link_offset_steps = 1

    dist = 0.01

    cx = 0.4
    cy = 0.6
    cz = 0.7
    
    camera_angle = 30
    link_offset = 0.0

    #x_range = np.linspace(cx-dist,cx+dist,nSteps)
    y_range = np.linspace(cy-dist,cy+dist,nSteps)
    z_range = np.linspace(cz-dist,cz+dist,nSteps)
    
    yz_coords = [(cy,cz),(cy-dist,cz), (cy+dist,cz),(cy,cz-dist),(cy,cz+dist)]

    occlusion_range = np.linspace(-0.05,0.05,occlusion_steps)
    orientation_range = np.linspace(-math.pi/4,math.pi/4,orientation_steps)

    radius_range = np.linspace(0.06,0.12,radius_steps)
    
    noise_range = np.linspace(0.001,0.01, noise_steps)
    #link_offset_range = np.linspace(0.01,0.05,link_offset_steps)
    #camera_angle_range = np.linspace(10,25,angle_steps)
    
    #plt.clf()
    #fig = plt.figure(2)
        
    #plt.ion()
    #plt.show()
    #plt.hold(True)
    weight_steps = 50
    weights = [(1.0,0.0),(0.8,0.2),(0.5,0.5)]
    w1 = np.linspace(1.0,0.0,weight_steps)
    w2 = np.linspace(0.0,1.0,weight_steps)
    
    mvs = move_to_see(9,"VREP", step_size=0.001, size_weight=0.8, manip_weight=0.2,end_tolerance=1.5,max_pixel=0.4)

    trials = [] 
    naive_trials = []
    
    #harvey_start_position = [-5.0032e-3,-1.8490e-1, +9.5281e-1]
    #harvey_start_orientation = [0, 0.0, -180.0]

    # print x_range
    #for x in x_range:
    
    path = 'C:\\Users\\Chris\\Dropbox\\Apps\\ShareLaTeX\\ICRA 2018 Move To See\\results\\sim_data\\experiment_3_varied_weights\\'
    
    occlusion_y = -0.05
    angle = -math.pi/4
    radius = 0.07
    noise_std = 0.0
    y = cy 
    z = cz
    
    count = 0
    for w in weights:
        #for radius in radius_range:
#            for y in y_range:
#                for z in z_range:
            #for yz in yz_coords:
                #for occlusion_y in occlusion_range:
                    #for angle in orientation_range: 
                        #for noise_std in noise_range:
                            
                            #y = yz[0]
                            #z = yz[1]
                                                 
                            mvs.interface.start_sim()
                            time.sleep(3)
                            #for link_offset in link_offset_range:
                                #for camera_angle in camera_angle_range:
                            mvs.setObjectiveFunctionWeights(w1[0],w2[1])
                            mvs.setNoiseStd(noise_std)
                            
                            #mvs.interface.initialise_sim(np.array([-0.3,0.0,0.6]))
                            #mvs.interface.initialise_sim(np.array([-0.3,0.0,0.5,-0.707,0.0,0.707,0.0]),frame="harvey")
                            
                            #harvey base start
                            #mvs.interface.initialise_sim(np.array([0.3,0.6,0.6,0.0,-0.707,0.0,-0.707,0.0]),frame="base_frame")
                            #mvs.interface.set_joints_degrees(np.array([0.0,-360,-270-45,-90,-135,-90,90]))
                            
                            #mvs.interface.set_object_position('harvey',harvey_start_position,harvey_start_orientation,-1)
                             
                            mvs.interface.init_joints()
                            
                            mvs.interface.set_object_position('capsicum',[cx,y,z],[],'base_frame')
                            mvs.interface.set_object_position('leaf_0',[cx-0.1,y+occlusion_y,z],[angle,0.0,0.0],'base_frame')
                            #mvs.interface.set_object_position('leaf_1',[x-0.1,y+occlusion,z],[angle,0.0,0.0],'base_frame')
                            
                            set_euler_angles = False
                            mvs.setCameraPosition(radius,link_offset,camera_angle,set_euler_angles)
        
                            data = {}
                            data = mvs.execute()
                            
                            data['capsicum_position'] = [cx,y,z]
                            data['occlusion_position'] = [cx-0.1,y+occlusion_y,z]
                            data['occlusion_angle'] = angle
                            
                            data['radius']=radius
                            data['weights'] = w
                            data['noise_std'] = noise_std
                               
                            trials.append(data)
                            
                            """mvs.interface.init_joints()
                            data2 = mvs.execute_naive()
                            data2['capsicum_position'] = [cx,y,z]
                            data2['occlusion_position'] = [cx-0.1,y+occlusion_y,z]
                            data2['occlusion_angle'] = angle
                            
                            data2['radius']= radius
                            data2['weights'] = w
                            
                            naive_trials.append(data2)"""     
                            
                            #time.sleep(2)     
                            mvs.interface.stop_sim()
                            time.sleep(1)
                            mvs.reset()
                            
                            file_name = "vrep_sim_"+ time.strftime("%Y_%m_%d-%H_%M_%S")

                            scipy.io.savemat(path + file_name + "_trial_"+str(count)+"_exp_1"+".mat", data)
                            #scipy.io.savemat(path + file_name + "_naive_trial_"+str(count)+"_exp_1"+".mat", data2)
                            
                            count = count + 1
                                
                 

    
    file_name = "vrep_sim_"+ time.strftime("%Y_%m_%d-%H_%M_%S")
    
    fileObject = open(path + file_name + ".pickle",'wb') 
    
    count = 1
    #for trial in trials:
    #    scipy.io.savemat(file_name+"_trial_"+str(count)+"_exp_1"+".mat", trial)
    #    scipy.io.savemat(file_name+"_naive_trial_"+str(count)+"_exp_1"+".mat", trial)
    #    count = count + 1
    
    pickle.dump([trials,naive_trials],fileObject)
    
    fileObject.close()
    
    print ('Program ended')

if __name__=="__main__":
    t1 = time.time()
    run()
    t2 = time.time()
    
    print ("Total Time to run Experiment: ", t2-t1)
    #cProfile.run('move_to_see()')
