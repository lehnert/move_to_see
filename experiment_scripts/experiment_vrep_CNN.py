###############################################################################
#
# V-REP Simulation Experiment: Convolutional Neural Network 3D Move To See
#
# Performs V-REP simulation of 'Harvey' robot moving its end effector towards 
# a capsicum fruit which is partially occluded by a leaf. The end effector is 
# moved to obtain an unoccluded view of the fruit in preparation for fruit
# harvesting.  This experiment compares two 3D Move To See (3DMTS) approaches:
# standard 3DMTS (utilising a 3D array of nine cameras to obtain an objective
# function gradient to determine direction of end effector movement) and
# Convolution Neural Network (CNN) based 3DMTS (utilises a single camera to 
# determine direction of end effector movement, trained on data from standard
# 3DMTS)
# 
# Created by:   Paul Zapotezny-Anderson
#               10 July 2019
#               Based on "experiment_vrep_vary_radius.py" by Chris Lehnert
# 
# Preliminaries: ensure that "move_to_see_experiment_harvey_simple.ttt" is
# executed prior to running this script (the .ttt file should load V-REP and 
# the Harvey simulation)
#
###############################################################################

#### Modules to import

from move_to_see_interfaces_CNN import move_to_see
import time
import numpy as np
import math
import scipy.io
import torch

#### Supporting Functions

# run() executes the 3DMTS experiment
def run():

    # capsicum position with respect to V-REP world frame
    cx = 0.4                
    cy = 0.6
    cz = 0.7
    
    # 3DMTS camera array parameters
    radius = 0.07               # size of 3DMTS camera array
    camera_angle = 30           # arrangement of cameras in array
    link_offset = 0.0           # camera array offset from end effector
    set_euler_angles = False    # ???
        
    # trial configuration
    trial_iterations = 1        # number of trials to be run
    trial_range = np.linspace(1,trial_iterations,trial_iterations)
    mvs = move_to_see(9,"VREP", step_size=0.001,            # MTS run conditions
                      size_weight=1.0, manip_weight=0.0,    # MTS objective funtion weights
                      end_tolerance=1.5,max_pixel=0.4)      # MTS termination conditions
    
    # prepare CNN model
    device = torch.device("cuda:1" if torch.cuda.is_available() else "cpu")     # rsetup CNN to run on GPU2 if available
    MTS_CNN_model = torch.load('..\CNN\MTSmodel3_2_3_ep50_bs64_j0_lr00001_lrd01_t0552_v1896.pt') # load finetuned CNN (targets: DirGrad x, y, z)
    MTS_CNN_model.to(device)                                                    # load CNN to processing device (GPU)

    # save location of trail data
    path = '..\\data\\VREP_CNN_trials_3_2\\'
    
    # iterate through number of trials requested
    for iteration in trial_range:
        
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
        
        # Perform CNN-based MTS                                    
        mvs.interface.start_sim()       # start sim - ensure V-REP scene is loaded first
        time.sleep(3)                   # allow time for V-REP to initiate sim

        # initialise positision of objects in scene
        mvs.interface.init_joints()
        #mvs.interface.set_object_position('capsicum',fruit_pos,[],'base_frame')
        #mvs.interface.set_object_position('leaf_0',leaf_pos,[occlusion_angle,0.0,0.0],'base_frame')
        mvs.interface.set_object_position('capsicum',fruit_pos_offset,[],'base_frame')
        mvs.interface.set_object_position('leaf_0',leaf_pos_offset,[occlusion_angle,0.0,0.0],'base_frame')
        mvs.setCameraPosition(radius,link_offset,camera_angle,set_euler_angles)

        # initiate CNN-based 3DMTS and collect end effector trajectory data
        CNN_data = {}                   # initialise as data dictionary
        CNN_data = mvs.execute_CNN(MTS_CNN_model,device)
        CNN_data['capsicum_position'] = fruit_pos
        CNN_data['occlusion_position'] = leaf_pos
        CNN_data['occlusion_angle'] = occlusion_angle
        CNN_data['radius']=radius
        CNN_data['offset']=[offset_x,offset_y,offset_z] # comment out if offset not required
           
        # stop V-REP simulation and reset MTS trajectory state data
        mvs.interface.stop_sim()
        time.sleep(1)
        mvs.reset()
        
        # Perform standard 3DMTS
        mvs.interface.start_sim()
        time.sleep(3)

        # initialise positision of objects in scene
        mvs.interface.init_joints()
        #mvs.interface.set_object_position('capsicum',fruit_pos,[],'base_frame')
        #mvs.interface.set_object_position('leaf_0',leaf_pos,[occlusion_angle,0.0,0.0],'base_frame')
        mvs.interface.set_object_position('capsicum',fruit_pos_offset,[],'base_frame')
        mvs.interface.set_object_position('leaf_0',leaf_pos_offset,[occlusion_angle,0.0,0.0],'base_frame')
        mvs.setCameraPosition(radius,link_offset,camera_angle,set_euler_angles)

        # initiate standard 3DMTS and collect end effector trajectory data
        MTS_data = {}                   # initialise as data dictionary
        MTS_data = mvs.execute()
        MTS_data['capsicum_position'] = fruit_pos
        MTS_data['occlusion_position'] = leaf_pos
        MTS_data['occlusion_angle'] = occlusion_angle
        MTS_data['radius']=radius
        MTS_data['offset']=[offset_x,offset_y,offset_z] # comment out if offset not required
        
        # save CNN and 3DMTS data as matlab files
        file_name = "vrep_sim_" + time.strftime("%Y_%m_%d-%H_%M_%S")
        CNN_name = "-CNN_steps_" + str(CNN_data['count'][-1])+".mat"
        MTS_name = "-MTS_steps_" + str(MTS_data['count'][-1])+".mat"
        scipy.io.savemat(path + file_name + CNN_name, CNN_data)
        scipy.io.savemat(path + file_name + MTS_name, MTS_data)
        
    print ('Program ended')

#### Main Function
    
if __name__=="__main__":    # only run if this module is run as the main module
    t1 = time.time()        # start process time
    run()                   # execute 3DMTS experiment
    t2 = time.time()        # end process time
    
    print ("Total Time to run Experiment: ", t2-t1) # display process time