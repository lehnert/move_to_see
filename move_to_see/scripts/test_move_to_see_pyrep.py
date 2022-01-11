#!/usr/bin/env python

import cv2
import numpy as np
import math
import time
import scipy.io
import pickle
from move_to_see import move_to_see
from pyrep_interface import pyrep_interface


# interface = pyrep_interface(9)
#
# interface.init_joints()

xy_offset = 0.03              # size of 3DMTS camera array in metres
z_offset = 0.03
camera_angle = 30           # angle polar coordinate in spherical coordinate calculation
link_offset = 0.0           # camera array offset from end effector

# capsicum position with respect to V-REP world frame
cx = 0.6 #0.4
cy = 0.6 #0.6
cz = 0.7

# set-up fruit position with random offset (represents noise in starting position of end effector)
# offset_x = np.random.uniform(-0.05,0.05)
# offset_y = np.random.uniform(-0.05,0.05)
# offset_z = np.random.uniform(-0.05,0.05)
offset_x = 0
offset_y = 0
offset_z = 0
fruit_pos = [cx, cy, cz]
fruit_pos_offset = [cx + offset_x,cy + offset_y,cz + offset_z]  # comment out if offset not required

# set-up occluding leaf in random position infront of fruit (with same random offset)
# occlusion_y = np.random.uniform(-0.06,0.06)
# occlusion_z = np.random.uniform(-0.08,0.08)
# occlusion_angle = np.random.uniform(-math.pi/2,math.pi/2)

occlusion_y = -0.06
occlusion_z = 0
occlusion_angle = 0

leaf_pos = [cx - 0.1,cy + occlusion_y,cz + occlusion_z]
leaf_pos_offset = [cx + offset_x - 0.1,cy + offset_y + occlusion_y,cz + offset_z + occlusion_z] # comment out if offset not required

mts = move_to_see(9,"SIM", step_size=0.001,            # MTS run conditions
                size_weight=1.0, manip_weight=0.0,    # MTS objective funtion weights
                end_tolerance=1.5,max_pixel=0.4, max_count=500)      # MTS termination conditions mts.initCameraPosition()



mts.interface.init_joints()

mts.interface.capsicum.set_position(fruit_pos_offset,relative_to=mts.interface.base_frame)
mts.interface.leaf_0.set_position(leaf_pos_offset[0:3],relative_to=mts.interface.base_frame)
mts.interface.leaf_0.set_orientation(leaf_pos_offset[3:6],relative_to=mts.interface.base_frame)
mts.setCameraPosition(xy_offset,z_offset,link_offset)

#step the sim once so all changes take effect
mts.interface.step_sim()

data = mts.execute(move_robot=True)
