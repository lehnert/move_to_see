
#class for creating and training DQN for move to see problem
#Author: chris lehnert
#####
import gym
import math
import DQN as dqn
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
import dqn_vrep_interface
import datetime

Transition = namedtuple('Transition',
                ('state', 'action', 'next_state', 'reward'))

from tensorboardX import SummaryWriter


class dqn_move_to_see():

    def __init__(self, env_interface, device):

        self.env = env_interface
        self.env.init()

        # self.BATCH_SIZE = 128
        self.BATCH_SIZE = 20
        self.GAMMA = 0.9
        self.EPS_START = 0.9
        self.EPS_END = 0.05
        self.EPS_DECAY = 200
        self.TARGET_UPDATE = 10

        self.is_ipython = 'inline' in matplotlib.get_backend()

        if self.is_ipython:
            from IPython import display

        plt.ion()

        self.device = device

        # if (interface == 'VREP'):
        #     self.env = dqn_vrep_interface()
        # elif (interface == 'ROS'):
        #     print("USING ROS INTERFACE WHICH ISN'T IMPLEMENTED")

        init_image = self.env.get_image()
        _, _, self.image_height, self.image_width = init_image.shape

        # Get number of actions from gym action space
        self.n_actions = self.env.n_actions
        
        time = datetime.datetime.now()
        self.name = '\\tensorboard_logs\\MoveToSeeDiscrete_{}-{}-{}_{}_{}'.format(time.month,time.day,time.hour,time.minute,time.second)
        self.path = os.getcwd()
        logdir = self.path+self.name
        self.writer = SummaryWriter(logdir=logdir)

        self.policy_net = dqn.network(self.image_height, self.image_width, self.n_actions).to(self.device)
        self.target_net = dqn.network(self.image_height, self.image_width, self.n_actions).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.RMSprop(self.policy_net.parameters())
        self.memory = dqn.ReplayMemory(10000)

        self.episode_durations = []
        self.episode_max_rewards = []
        self.steps_done = 0
        self.episodes_done = 0
        self.max_reward = 0
    
    def show_image(self, image):
        plt.figure(1)
        plt.imshow(image.cpu().squeeze(0).permute(1, 2, 0).numpy(),
           interpolation='none')
        plt.title('Example extracted image')
        plt.show()
        plt.pause(0.001)

    def plot_durations(self):

        plt.figure(2)
        plt.clf()
        durations_t = torch.tensor(self.episode_durations, dtype=torch.float)
        plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('Duration')
        plt.plot(durations_t.numpy())
        # Take 100 episode averages and plot them too
        # if len(durations_t) >= 100:
        #     means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
        #     means = torch.cat((torch.zeros(99), means))
        #     plt.plot(means.numpy())

        plt.pause(0.001)  # pause a bit so that plots are updated
        if self.is_ipython:
            display.clear_output(wait=True)
            display.display(plt.gcf())

    def plot_rewards(self):

        plt.figure(3)
        plt.clf()
        episode_rewards = torch.tensor(self.episode_max_rewards, dtype=torch.float)
        plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('Rewards')
        plt.plot(episode_rewards.numpy())
        # Take 100 episode averages and plot them too
        if len(episode_rewards) >= 100:
            means = episode_rewards.unfold(0, 100, 1).mean(1).view(-1)
            means = torch.cat((torch.zeros(99), means))
            plt.plot(means.numpy())

        plt.pause(0.001)  # pause a bit so that plots are updated
        if self.is_ipython:
            display.clear_output(wait=True)
            display.display(plt.gcf())

    def select_action(self, state):
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.episodes_done / self.EPS_DECAY)
        # self.steps_done += 1
        if sample > eps_threshold:
            print('using network to select action')
            with torch.no_grad():
                # t.max(1) will return largest column value of each row.
                # second column on max result is index of where max element was
                # found, so we pick action with the larger expected reward.
                return self.policy_net(state).max(1)[1].view(1, 1)
        else:
            print('taking random action')
            return torch.tensor([[random.randrange(self.n_actions)]], device=self.device, dtype=torch.long)


    def train(self, num_episodes=50):
        
        for self.episodes_done in range(num_episodes):
            i_episode = self.episodes_done
            # Initialize the environment and state
            print('Episode: ', i_episode)
            self.env.reset()
            self.max_reward_in_episode = 0

            last_image = self.env.get_image()
            current_image = self.env.get_image()

            self.state = current_image - last_image            
            for t in count():
                print('Step: ', t)
                # Select and perform an action
                self.action = self.select_action(self.state)

                reward, done = self.env.step(self.action.item(), 0.05)

                self.reward = torch.tensor([reward], device=self.device)
                
                if reward > self.max_reward_in_episode:
                    self.max_reward_in_episode = reward

                # Observe new state
                last_image = current_image
                current_image = self.env.get_image()
                self.show_image(current_image)
                if not done:
                    self.next_state = current_image - last_image
                else:
                    self.next_state = None

                # Store the transition in memory
                self.memory.push(self.state, self.action, self.next_state, self.reward)

                # Move to the next state
                self.state = self.next_state

                # Perform one step of the optimization (on the target network)
                self.optimize_model()

                
                
                if done:
                    self.writer.add_scalar('Monitoring/EpisodeLength', t+1, i_episode)
                    self.writer.add_scalar('Monitoring/MaxReward', self.max_reward_in_episode, i_episode)
                    # self.writer.add_image('Monitoring/ImageFinal', current_image, i_episode)
                    # self.episode_max_rewards.append(self.max_reward_in_episode)
                    # self.episode_durations.append(t + 1)
                    # self.plot_durations()
                    # self.plot_rewards()
                    break
            # Update the target network, copying all weights and biases in DQN
            if i_episode % self.TARGET_UPDATE == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())

        print('Complete')
        self.env.close()
        print('Closed')
        # plt.ioff()
        # plt.show()

    def optimize_model(self):
        if len(self.memory) < self.BATCH_SIZE:
            return
        transitions = self.memory.sample(self.BATCH_SIZE)
        # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
        # detailed explanation). This converts batch-array of Transitions
        # to Transition of batch-arrays.
        batch = Transition(*zip(*transitions))

        # Compute a mask of non-final states and concatenate the batch elements
        # (a final state would've been the one after which simulation ended)
        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                            batch.next_state)), device=self.device, 
                                            dtype=torch.uint8)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                                    if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
        # columns of actions taken. These are the actions which would've been taken
        # for each batch state according to policy_net
        state_action_values = self.policy_net(state_batch).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        # Expected values of actions for non_final_next_states are computed based
        # on the "older" target_net; selecting their best reward with max(1)[0].
        # This is merged based on the mask, such that we'll have either the expected
        # state value or 0 in case the state was final.
        next_state_values = torch.zeros(self.BATCH_SIZE, device=self.device)
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
        # Compute the expected Q values
        expected_state_action_values = (next_state_values * self.GAMMA) + reward_batch

        # Compute Huber loss
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

    def saveModel(self, PATH):

        filename = '\mvs_model.pt'
        torch.save(self.policy_net.state_dict(), PATH+filename)

    def loadModel(self, PATH):

        filename = '\mvs_model.pt'
        self.policy_net.load_state_dict(torch.load(PATH+filename))


if __name__ == "__main__":
    import os
    path = os.getcwd()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    VREPENV = dqn_vrep_interface.vrep_env(device)

    DQNMVS = dqn_move_to_see(VREPENV, device)

    # dqn_mvs.loadModel(path)

    DQNMVS.train(500)

    print("Saving model to: ", path)
    DQNMVS.saveModel(path)
