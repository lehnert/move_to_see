import gym
import gym_pyrep

import math
import numpy as np

from stable_baselines.common.env_checker import check_env

import random

from collections import namedtuple
from itertools import count

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

from torch.utils.tensorboard import SummaryWriter
# from tensorboardX import SummaryWriter\

writer = SummaryWriter()

# from baselines import deepq

# import matplotlib
# import matplotlib.pyplot as plt

from PIL import Image


Transition = namedtuple('Transition',
                ('state', 'action', 'next_state', 'reward'))


class DQN(nn.Module):

    def __init__(self, h, w, outputs, device):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)

        self.device = device

        # Number of Linear input connections depends on output of conv2d layers
        # and therefore the input image size, so compute it.        
        self.convw = self.conv2d_size_out(self.conv2d_size_out(self.conv2d_size_out(w)))
        self.convh = self.conv2d_size_out(self.conv2d_size_out(self.conv2d_size_out(h)))
        
        self.linear_input_size = self.convw * self.convh * 32
        self.head = nn.Linear(self.linear_input_size, outputs)

    def conv2d_size_out(self, size, kernel_size = 5, stride = 2):
            return (size - (kernel_size - 1) - 1) // stride  + 1

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = x.to(self.device)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))

class ReplayMemory(object):

    def __init__(self, capacity):
        
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


############################


class agent():

    def __init__(self, gym_env, device):

        self.BATCH_SIZE = 128 #128
        self.GAMMA = 0.999
        self.EPS_START = 0.9
        self.EPS_END = 0.05
        self.EPS_DECAY = 200
        self.TARGET_UPDATE = 10
        self.MAX_STEPS = 40
        self.gym_env = gym_env
        self.device = device

        # Get screen size so that we can initialize layers correctly based on shape
        # returned from AI gym. Typical dimensions at this point are close to 3x40x90
        # which is the result of a clamped and down-scaled render buffer in get_screen()
        self.init_image = self.get_observation()
        # self.image_height, self.image_width, _ = self.init_image.shape
        _, _, self.image_height, self.image_width = self.init_image.shape

        # Get number of actions from gym action space
        self.n_actions = self.gym_env.action_space.n

        self.policy_net = DQN(self.image_height, self.image_width, self.n_actions, self.device).to(self.device)
        self.target_net = DQN(self.image_height, self.image_width, self.n_actions, self.device).to(self.device)
        
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.RMSprop(self.policy_net.parameters())
        self.memory = ReplayMemory(10000)

        self.optimisation_step = 0
        self.steps_done = 0
        self.episode_average_reward = []
        self.running_loss = 0

    def get_observation(self):

        resize = T.Compose([T.ToPILImage(),
                    T.Resize(40, interpolation=Image.CUBIC),
                    T.ToTensor()])

        image = self.gym_env.render().transpose((2, 0, 1))
        # Cart is in the lower half, so strip off the top and bottom of the screen
        # _, image_height, image_width = image.shape

        # screen = screen[:, int(image_height*0.4):int(image_height * 0.8)]
        # view_width = int(image_width * 0.6)

        # cart_location = get_cart_location(image_width)

        # if cart_location < view_width // 2:
        #     slice_range = slice(view_width)
        # elif cart_location > (image_width - view_width // 2):
        #     slice_range = slice(-view_width, None)
        # else:
        #     slice_range = slice(cart_location - view_width // 2,
        #                         cart_location + view_width // 2)
        # # Strip off the edges, so that we have a square image centered on a cart
        # screen = screen[:, :, slice_range]
        # Convert to float, rescale, convert to torch tensor
        # (this doesn't require a copy)

        image = np.ascontiguousarray(image, dtype=np.float32) / 255
        image = torch.from_numpy(image)
        # Resize, and add a batch dimension (BCHW)
        return resize(image).unsqueeze(0)

    def select_action(self,state):

        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.steps_done / self.EPS_DECAY)
        self.steps_done += 1
        if sample > eps_threshold:
            with torch.no_grad():
                # t.max(1) will return largest column value of each row.
                # second column on max result is index of where max element was
                # found, so we pick action with the larger expected reward.
                return self.policy_net(state).max(1)[1].view(1, 1)
        else:
            return torch.tensor([[random.randrange(self.n_actions)]], device=self.device, dtype=torch.long)



    def plot(self):
        plt.figure(2)
        plt.clf()
        episode_average_reward = torch.tensor(self.episode_average_reward, dtype=torch.float)
        plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('Reward')
        plt.plot(episode_average_reward.numpy())
        # Take 100 episode averages and plot them too
        if len(episode_average_reward) >= 100:
            means = episode_average_reward.unfold(0, 100, 1).mean(1).view(-1)
            means = torch.cat((torch.zeros(99), means))
            plt.plot(means.numpy())

        plt.pause(0.001)  # pause a bit so that plots are updated
        if is_ipython:
            display.clear_output(wait=True)
            display.display(plt.gcf())


    def optimize_model(self, step, episode):
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
                                            batch.next_state)), device=self.device, dtype=torch.bool)
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
        criterion = nn.SmoothL1Loss()
        loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))
        writer.add_scalar("Loss/train", loss, self.optimisation_step)
        self.optimisation_step += 1
        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()

        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

        # self.running_loss += loss.item()
        # writer.add_scalar('training loss', self.running_loss/step, step*episode)

    def train(self, num_episodes=50):

        for i_episode in range(num_episodes):
            # Initialize the environment and state
            print("Episode: ", i_episode)
            self.gym_env.reset()
            last_observation = self.get_observation()
            current_observation = self.get_observation()
            state = current_observation - last_observation

            avg_reward = 0
            max_reward = 0
            
            #roll out episode
            for n_steps in count():
                # Select and perform an action
                # Observe next state

                last_observation = current_observation

                action = self.select_action(state)
                _, reward, done, _ = self.gym_env.step(action.item())
                current_observation = self.get_observation()

                avg_reward += avg_reward + reward
                if reward > max_reward:
                    max_reward = reward

                reward = torch.tensor([reward], device=self.device)
                

                # writer.add_image('observation', current_observation[0,:,:,:], n_steps*(i_episode+1), dataformats='CHW')
                # writer.add_scalar('reward',reward, n_steps*(i_episode+1))

                #remember last observation
                                
                if not done:
                    next_state = current_observation - last_observation
                else:
                    next_state = None

                # Store the transition in memory
                self.memory.push(state, action, next_state, reward)

                # Move to the next state
                state = next_state

                # Perform one step of the optimization (on the policy network)
                self.optimize_model(n_steps, i_episode)

                if done or n_steps > self.MAX_STEPS:
                    writer.add_scalar('Average Reward', avg_reward/(n_steps + 1), i_episode)
                    writer.add_scalar('Final Reward',reward, i_episode)
                    writer.add_scalar('Max Reward',max_reward, i_episode)
                    writer.add_image('observation', current_observation[0,:,:,:], i_episode, dataformats='CHW')
                    self.episode_average_reward.append(avg_reward/n_steps)
                    # self.plot()
                    break
            
            print("episode finished with steps: ", n_steps)
            # Update the target network, copying all weights and biases in DQN
            if i_episode % self.TARGET_UPDATE == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())


            writer.flush()

# model = DQN('MlpPolicy', env, learning_rate=1e-3, prioritized_replay=True, verbose=1)
# model.learn(total_timesteps=int(2e5))

def main():

    gym_env = gym.make('pyrep-v0')

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    agent_ = agent(gym_env, device)

    agent_.train(num_episodes=500)





if __name__ == "__main__":

    main()