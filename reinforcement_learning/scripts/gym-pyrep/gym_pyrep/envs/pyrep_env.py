import gym
from gym import spaces

import sys
sys.path.insert(0, "~/mts_ws/src/move_to_see/move_to_see/scripts/")

from pyrep_interface import pyrep_interface

class PyrepEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.interface = pyrep_interface(number_of_cameras, '~/vrep_scenes/PyRep_harvey.ttt')
        
        self.image_width = self.interface.image_width
        self.image_height = self.interface.image_width
        self.image_cx = round(self.image_width/2)
        self.image_cy = round(self.image_height/2)
        self.FOV_x = 60
        self.FOV_y = 60

    def step(self, action):

        servoPoseEuler(self, delta_pose)
        self.interface.step_sim()
        self.interface.getObjectiveFunctionValues(False)

    def reset(self):
        self.stop_sim()
        self.start_sim()

    def render(self, mode='human', close=False):
        self.interface.displayImage(self.interface.getImage(4))