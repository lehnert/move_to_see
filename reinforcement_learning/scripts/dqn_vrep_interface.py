
import sys
import os

# path = os.getcwd()
# move_to_see_package_path = '\move_to_see\scripts'
path = '..\..\scripts'
print("adding to path: ", path)
sys.path.insert(0, path)

import vrep_interface

import gym
import time
import math

#####
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
#####

from collections import namedtuple
from itertools import count
from PIL import Image

#####
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T
####
from enum import Enum
class action(Enum):
    up = 1
    down = 2
    left = 3
    right = 4
    forward = 5
    backward = 6
    roll_up = 7
    roll_down = 8
    pitch_up = 9
    pitch_down = 10


#####

class vrep_env(vrep_interface.vrep_interface):

    def __init__(self, device):
        self.n_cameras = 9
        self.ref_camera_index = 4
        
        self.steps = 0
        # super().__init__(self.n_cameras,ip='127.0.0.1', port=20001)
        super().__init__(self.n_cameras,ip='127.0.0.1', port=19997)
        self.state = []

        self.resize = T.Compose([T.ToPILImage(),
                    T.Resize(40, interpolation=Image.CUBIC),
                    T.ToTensor()])
        
        self.n_actions = 10
        self.max_steps = 20
        self.max_reward = 0.6
        self.device = device
        self.init_reward = 0

        _, _, self.image_height, self.image_width = self.get_image().shape

        self.actions = {'up':1,
                        'down':2,
                        'left' : 3,
                        'right' : 4,
                        'forward' : 5,
                        'backward' : 6,
                        'roll_up' : 7,
                        'roll_down': 8,
                        'pitch_up' : 9,
                        'pitch_down' : 10}

    def init(self):
        super().start_sim()
        time.sleep(4)
        print("Initalised Joints: ret code: ",super().init_joints())

        print("Current Pose: ", super().getCurrentPose())
        cx = 0.4
        cy = 0.6
        cz = 0.7
        occlusion_y = -0.05
        angle = 30
        leaf_angle = -math.pi/4
        super().set_object_position('capsicum',[cx,cy,cz],[],'base_frame')
        super().set_object_position('leaf_0',[cx-0.1,cy+occlusion_y,cz],[leaf_angle,0.0,0.0],'base_frame')
        super().setCameraOffsets(radius=0.07,link_offset=0.0,angle=30)
        
        #get first reward in order to set init reward to zero
        # pixel_sizes, blob_centres, manip = super().getObjectiveFunctionValues()
        # self.init_reward = pixel_sizes[self.ref_camera_index]
        # self.init_reward = 0

    def get_image(self):
        image = self.getImage(self.ref_camera_index)

        # Returned screen requested by gym is 400x600x3, but is sometimes larger
        # such as 800x1200x3. Transpose it into torch order (CHW).
        #print(image.shape)
        image_transpose = image.transpose((2, 0, 1))
        image_transpose = np.ascontiguousarray(image_transpose, dtype=np.float32) / 255
        image_transpose = torch.from_numpy(image_transpose)
        # Resize, and add a batch dimension (BCHW)
        return self.resize(image_transpose).unsqueeze(0).to(self.device)

    def step(self, action, step_size):
        
        deltaPose = np.array([0,0,0,0,0,0])
        angle_scale = 2

        if action==self.actions['up']:
            deltaPose = deltaPose + [step_size,0,0,0,0,0]

        if action==self.actions['down']:
            deltaPose = deltaPose + [-step_size,0,0,0,0,0]

        if action==self.actions['left']:
            deltaPose = deltaPose + [0,step_size,0,0,0,0]

        if action==self.actions['right']:
            deltaPose = deltaPose + [0,-step_size,0,0,0,0]

        if action==self.actions['forward']:
            deltaPose = deltaPose + [0,0,step_size,0,0,0]

        if action==self.actions['backward']:
            deltaPose = deltaPose + [0,0,-step_size,0,0,0]

        if action==self.actions['roll_up']:
            deltaPose = deltaPose + [0,0,0,0,step_size/angle_scale,0]

        if action==self.actions['roll_down']:
            deltaPose = deltaPose + [0,0,0,0,-step_size/angle_scale,0]

        if action==self.actions['pitch_up']:
            deltaPose = deltaPose + [0,0,0,step_size/angle_scale,0,0]

        if action==self.actions['pitch_down']:
            deltaPose = deltaPose + [0,0,0,-step_size/angle_scale,0,0]

        #append 0 to end for final rotation not used
        # dRoll, dPitch = self.compute_roll_pitch(self.x_ref.blob_centre)
        
        # pose_delta = np.append(pose_delta,[-dPitch/2, -dRoll/2, 0]) #add 0 angle for euler x
        # deltaPose = np.append(deltaPose,[0,0,0]) 
        deltaPose = deltaPose.reshape((6,))
        if not super().servoPoseEuler(deltaPose):
            print("Failed to move to pose")
            return 0.0, True 

        success, pixel_sizes, blob_centres, manip = super().getObjectiveFunctionValues()
        #num_pixels = self.image_height*self.image_width
        # norm_pixel_sizes = np.array(pixel_sizes)/num_pixels
        if success:
            reward = pixel_sizes[self.ref_camera_index]
            # reward = pixel_sizes[self.ref_camera_index] - self.init_reward
            print("Current Reward: ", reward)
            self.steps += 1
            done = False
            if reward > self.max_reward:
                print('Reward maximised: done')
                done = True
            if self.steps > self.max_steps:
                print('Steps exceeded max: done')
                done = True

            return reward, done
        else:
            print("Failed to get reward")
            return 0.0, True

    def close(self):
        super().stop_sim()

    def reset(self):
        super().init_joints()
        self.steps = 0
        
        # super().stop_sim()
        # time.sleep(1)
        # self.init()




