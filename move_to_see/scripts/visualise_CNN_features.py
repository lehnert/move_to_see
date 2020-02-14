#!/usr/bin/env python
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

import time
import numpy as np
import math
import scipy.io
import pickle

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader

import torchvision.transforms as transforms
import torchvision.datasets as datasets

import matplotlib.pyplot as plt


import cv2

trans_x_train = transforms.Compose([
        #transforms.ToPILImage(),                               # disable if jitter is not needed
        #transforms.RandomAffine(0,translate=(jitter,jitter)),  # disable if jitter is not needed
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],    
                             std=[0.229, 0.224, 0.225])]) # as per resnet18 

def prep_input(input_data, input_transform):
    num = np.size(input_data,0)                 # number of images to processes
    trans_input = torch.empty(num,3,224,224)    # initialise return tensor
    for i in range(num):                        # process each input image
        img = np.squeeze(input_data[i,:,:,:])       # grab image from input array
        trans_input[i,:,:,:] = input_transform(img) # perform transforms on image
    return trans_input   


if __name__=="__main__":


    # self.interface.servoPose(pose_down, "harvey_arm", "pi_camera_link", 0.2)

    # prepare CNN model

    MTSdataset6 = scipy.io.loadmat("C:\\Users\\lehnert\\Documents Local\\move_to_see_data\\MTSdataset_6")

    device_type = "cuda:1" if torch.cuda.is_available() else "cpu"
    
    device = torch.device(device_type)     # rsetup CNN to run on GPU2 if available
    MTS_CNN_model6attn = torch.load('C:\\Users\\lehnert\\Documents Local\\move_to_see_data\\MTSmodel6attn_t06812_v26430.pt', map_location=device_type)
    MTS_CNN_model6 = torch.load('C:\\Users\\lehnert\\Documents Local\\move_to_see_data\\MTSmodel6_t05432_v29706.pt', map_location=device_type) # load finetuned CNN (targets: DirGrad x, y, z)
    # MTS_CNN_model.to(device)                                                    # load CNN to processing device (GPU)


    # image, label = foolbox.utils.imagenet_example(data_format="channels_first")
    # image = image/255

    x_trn_data = MTSdataset6['Xtrain']
    # test_image = image_sequence[:,:,:,1]

    x_trn_data = np.transpose(x_trn_data,(3,0,1,2))
    image_sequence_prepped = prep_input(x_trn_data, trans_x_train)
    test_image = image_sequence_prepped[100,:,:,:]
    test_image = torch.unsqueeze(test_image, 0)

    # image = torch.from_numpy(test_image)
    # image = torch.unsqueeze(image, 0)
    # print(image.shape)

    # remove last fully connected layer
    # resnet18 = models.resnet18(pretrained=True)
    # MTS_CNN_model

    MTS_CNN_model6 = list(MTS_CNN_model6.children())[:-9]
    MTS_CNN_model6 = nn.Sequential(*MTS_CNN_model6)
    for p in MTS_CNN_model6.parameters():
        p.requires_grad = False

    MTS_CNN_model6attn = list(MTS_CNN_model6attn.children())[:-9]
    MTS_CNN_model6attn = nn.Sequential(*MTS_CNN_model6attn)
    for p in MTS_CNN_model6attn.parameters():
        p.requires_grad = False


    out6 = MTS_CNN_model6(test_image)
    out6attn = MTS_CNN_model6attn(test_image)
    out6 = torch.squeeze(out6, 0)
    out6attn = torch.squeeze(out6attn, 0)

    plt.figure(1)
    for i in range(0,64):
        feature = out6[i].numpy()
        plt.subplot(8,8,i+1)
        plt.imshow(feature)

    plt.figure(2)
    for i in range(0,64):
        feature = out6attn[i].numpy()
        plt.subplot(8,8,i+1)
        plt.imshow(feature)
       
    plt.show()