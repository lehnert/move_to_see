import gym
from gym import spaces
import numpy as np

import sys
#this needs fixing to work on different machines
sys.path.append("/home/chris/mts_ws/src/move_to_see/")

from move_to_see.scripts.pyrep_interface import pyrep_interface

class PyrepEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.interface = pyrep_interface(9, '/home/chris/vrep_scenes/PyRep_harvey.ttt', headless=False)
        
        self.image_width = self.interface.image_width
        self.image_height = self.interface.image_width
        self.image_cx = round(self.image_width/2)
        self.image_cy = round(self.image_height/2)
        self.FOV_x = 60
        self.FOV_y = 60
        self.steps = 0

        self.step_size = 0.1
        self.max_reward = 1
        self.max_steps = 100

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

        self.N_DISCRETE_ACTIONS = 10

        self.action_space = spaces.Discrete(self.N_DISCRETE_ACTIONS)

        self.observation_space = spaces.Box(low=0, high=255,
                                        shape=(self.image_width, self.image_height, 3), dtype=np.uint8)

    def step(self, action = None):

        deltaPose = np.array([0,0,0,0,0,0])
        angle_scale = 2
        info = {}

        if action==self.actions['up']:
            deltaPose = deltaPose + [self.step_size,0,0,0,0,0]

        if action==self.actions['down']:
            deltaPose = deltaPose + [-self.step_size,0,0,0,0,0]

        if action==self.actions['left']:
            deltaPose = deltaPose + [0,self.step_size,0,0,0,0]

        if action==self.actions['right']:
            deltaPose = deltaPose + [0,-self.step_size,0,0,0,0]

        if action==self.actions['forward']:
            deltaPose = deltaPose + [0,0,self.step_size,0,0,0]

        if action==self.actions['backward']:
            deltaPose = deltaPose + [0,0,-self.step_size,0,0,0]

        if action==self.actions['roll_up']:
            deltaPose = deltaPose + [0,0,0,0,self.step_size/angle_scale,0]

        if action==self.actions['roll_down']:
            deltaPose = deltaPose + [0,0,0,0,-self.step_size/angle_scale,0]

        if action==self.actions['pitch_up']:
            deltaPose = deltaPose + [0,0,0,self.step_size/angle_scale,0,0]

        if action==self.actions['pitch_down']:
            deltaPose = deltaPose + [0,0,0,-self.step_size/angle_scale,0,0]

        deltaPose = deltaPose.reshape((6,))

        if not self.interface.servoPoseEuler(deltaPose):
            print("Failed to move to pose")
            observation = self.interface.getImage(4)  
            done = True
            reward = 0.0
            return observation, reward, done, info
        
        self.interface.step_sim()

        # success, pixel_sizes,_ = self.interface.getObjectiveFunctionValues()
        success, pixel_sizes_filtered, blob_centres, pixel_sizes, image_array, objects = self.interface.getObjectiveFunctionValues(show_image=False)

        observation = self.interface.getImage(4)    

        #num_pixels = self.image_height*self.image_width
        # norm_pixel_sizes = np.array(pixel_sizes)/num_pixels
        done = False
        if success:
            reward = pixel_sizes[4]
            # reward = pixel_sizes[self.ref_camera_index] - self.init_reward
            print("Current Reward: ", reward)
            self.steps += 1
            
            if reward > self.max_reward:
                print('Reward maximised: done')
                done = True
            if self.steps > self.max_steps:
                print('Steps exceeded max: done')
                done = True
                self.steps = 0

        else:
            print("Failed to get reward")
            reward = 0.0

        return observation, reward, done, info



    def reset(self):

        self.steps = 0

        self.interface.stop_sim()
        self.interface.start_sim()
        self.interface.step_sim()
        self.interface.step_sim()
        return self.interface.getImage(4)


    def render(self, mode='human', close=False):

        self.image = self.interface.getImage(4)
        self.interface.displayImage(self.image)

        return self.image
        