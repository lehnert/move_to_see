#!/usr/bin/env python

import cv2
import numpy as np
from numpy import pi
import math
import time
import scipy.io
import pickle
import os
from move_to_see import move_to_see
from pyrep_interface import pyrep_interface
from matplotlib import pyplot as plt


# interface = pyrep_interface(9)
#
# interface.init_joints()

xy_offset = 0.03              # size of 3DMTS camera array in metres
z_offset = 0.03
camera_angle = 30           # angle polar coordinate in spherical coordinate calculation
link_offset = 0.0           # camera array offset from end effector


mts = move_to_see(9,"SIM", step_size=0.001,            # MTS run conditions
                size_weight=1.0, manip_weight=0.0,    # MTS objective funtion weights
                end_tolerance=1.5,max_pixel=0.4, max_count=500)      # MTS termination conditions mts.initCameraPosition()

# set-up fruit position with random offset (represents noise in starting position of end effector)
offmaxx = 0.25 # 0.05
offset_x = np.random.uniform(0,0.2) # if it's too far away (+dx) MTS doesn't run
offset_y = np.random.uniform(-0.2,0.2)
offset_z = np.random.uniform(-0.15,0.15)
offset = np.array([offset_x, offset_y, offset_z])
# offset_x = 0
# offset_y = 0
# offset_z = 0
fruit_pos = mts.interface.capsicum.get_position() # current position
# print(fruit_pos)
fruit_pos_offset = fruit_pos + offset  # comment out if offset not required
# print(fruit_pos_offset)
# set-up occluding leaf in random position infront of fruit (with same random offset)
# occlusion_x = 0.1
occlusion_y = -0.06
occlusion_z = 0
occlusion_angle = 0
occlusion = np.array([-0.1, occlusion_y, occlusion_z])

leaf_pos = fruit_pos + occlusion
leaf_pos_offset = fruit_pos_offset + occlusion # comment out if offset not required

# specific trial to replicate (dark maroon trial / trial 20)
fruit_pos_offset = [-0.09877131, -0.42890756,  1.10698323]
leaf_pos_offset = [-0.19877131, -0.48890756,  1.10698323]

mts.interface.capsicum.set_position(fruit_pos_offset)#,relative_to=mts.interface.base_frame)
mts.interface.leaf_0.set_position(leaf_pos_offset[0:2])#,relative_to=mts.interface.base_frame)
# mts.interface.leaf_0.set_orientation(leaf_pos_offset[3:6],relative_to=mts.interface.base_frame)
mts.setCameraPosition(xy_offset,z_offset,link_offset)
mts.interface.init_joints()

# comment out below 3 lines if using harvey2
# mts.interface.capsicum.set_position(fruit_pos_offset,relative_to=mts.interface.base_frame)
# mts.interface.leaf_0.set_position(leaf_pos_offset[0:3],relative_to=mts.interface.base_frame)
# mts.interface.leaf_0.set_orientation(leaf_pos_offset[3:6],relative_to=mts.interface.base_frame)
mts.setCameraPosition(xy_offset,z_offset,link_offset)

# save capsicum + leaf positions across runs (in conjunction with loop.sh)
# ext = ".pkl"
# path = os.getcwd() # get path to current directory
# contents = os.listdir(path+'/pickles') # get file list
# files = [file for file in contents if file.endswith(ext)] # remove filenames with the wrong extension from list
# if not files: next = '1'
# else: 
#     fnames = ['.'.join(file.split('.')[:-1]) for file in files] # remove extensions
#     nums = [int('_'.join(x.split('_')[len(x.split('_'))-1:])) for x in fnames] # get the digits at the end of the filenames
#     nums.sort()
#     next = str(nums[-1]+1)
# data = {
#     "fruit_pos": fruit_pos_offset, 
#     "leaf_pos": leaf_pos_offset,
#     "fruit_pos_rel2world": mts.interface.capsicum.get_position(),
#     "leaf_pos_rel2world": mts.interface.leaf_0.get_position(),
#     "fruit_pos_rel2base": mts.interface.capsicum.get_position(relative_to=mts.interface.base_frame),
#     "leaf_pos_rel2base": mts.interface.leaf_0.get_position(relative_to=mts.interface.base_frame)
#     }
# pickle.dump(data, open("pickles/goalpos_" + next + '.pkl', 'wb'))

mts.interface.step_sim()

print("Initial Fruit Pos: ")
data = mts.execute(move_robot=True)
mts.interface.stop_sim()
mts.interface.disconnect_sim()