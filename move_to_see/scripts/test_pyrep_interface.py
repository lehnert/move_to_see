#!/usr/bin/env python
from pyrep_interface import pyrep_interface
import cv2
import numpy as np
import math


interface = pyrep_interface(9)

interface.init_joints()

radius = 0.07               # size of 3DMTS camera array
camera_angle = 30           # arrangement of cameras in array
link_offset = 0.0           # camera array offset from end effector

# capsicum position with respect to V-REP world frame
cx = 0.4
cy = 0.6
cz = 0.7

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




interface.setCameraOffsets(radius,link_offset,camera_angle)

#setup target and occluder
interface.capsicum.set_position(fruit_pos_offset, interface.base_frame)
interface.leaf_0.set_position(leaf_pos_offset, interface.base_frame)
interface.leaf_0.set_orientation([occlusion_angle,0.0,0.0], interface.base_frame)

# try:
while True:
    # interface.getImage(0)
    interface.getObjectiveFunctionValues()
    interface.pr.step()
# except KeyboardInterrupt:
# except:
#     print("Keyboard Pressed, exiting")
#     interface.pr.shutdown()
     # The answer was in the question!
