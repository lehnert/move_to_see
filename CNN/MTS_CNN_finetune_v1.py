# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 13:20:18 2019

@author: Paul Zapotezny-Anderson

This script trains a 3D Move-To-See (MTS) CNN "MTSmodel" to guide a robot arm end-effector
around objects that occlude a view of fruit to obtain a better view of the fruit.  
Training is done via fine-tuning a pre-trained CNN (ResNet18 trained on ImageNet 
image set) using training data from V-REP simulation images with targets (direction
gradient, delta pitch and delta roll) of 3DMTS V-REP simulation runs.
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
from torchvision import transforms, models
import scipy.io as spio
import numpy as np
import matplotlib.pyplot as plt
import time
from datetime import timedelta

start = time.time()     # get starting time of run - for process time

#### FUNCTIONS

# performs transforms on input data (array of images)
def prep_input(input_data, input_transform):
    num = np.size(input_data,0)                 # number of images to processes
    trans_input = torch.empty(num,3,224,224)    # initialise return tensor
    for i in range(num):                        # process each input image
        img = np.squeeze(input_data[i,:,:,:])       # grab image from input array
        trans_input[i,:,:,:] = input_transform(img) # perform transforms on image
    return trans_input                          # return tensor
        
# User-definable parameters
targetsize = 3          # number of targets to use: all = 5, dir grad only = 3
#jitter = 1/224          # pixels per image size for jitter (disbale if not needed)
epochs = 50             # number of epochs for training
batchsize = 64          # number of examples per mini-batch
init_lr = 0.00001       # initial learning rate
lr_drop = 0.01           # amount the learning rate is reduced
lr_dropperiod = 25      # number of epochs until drop in learning rate
opt_momentum = 0.9      # momentum for stochastic gradient descent

# User-configurable features
MTSmodel = models.resnet18(pretrained=True)
MTSmodel.fc = nn.Linear(512,targetsize)         # modify last FC-layer to output targets
loss_func = nn.MSELoss(reduction = 'sum')       # sum square error of each mini-batch
optimiser = optim.SGD(MTSmodel.parameters(), lr=init_lr, momentum=opt_momentum)
scheduler = optim.lr_scheduler.StepLR(optimiser, lr_dropperiod, gamma=lr_drop)
device = torch.device("cuda:1" if torch.cuda.is_available() else "cpu")

# Define dataset transforms
trans_x_train = transforms.Compose([
        #transforms.ToPILImage(),                               # disable if jitter is not needed
        #transforms.RandomAffine(0,translate=(jitter,jitter)),  # disable if jitter is not needed
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],    
                             std=[0.229, 0.224, 0.225])]) # as per resnet18 

trans_x_val = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],    
                             std=[0.229, 0.224, 0.225])]) # as per resnet18

# load matlab prepared VREP dataset
MTSdata =spio.loadmat('..\data\MTSdataset_3_2.mat')

# Extract training and validation data for conversion to PyTorch CNN training
# get training feature set (images, 224x224 BGR)
x_trn_data = MTSdata.get('Xtrain')              # (H x W x C x IDX)
x_trn_data = np.transpose(x_trn_data,(3,0,1,2)) # move image index to first dimension
x_train = prep_input(x_trn_data, trans_x_train) # transform input to tensor for training

# get training target set (dir grad [x, y, z], del pitch, del roll)
y_train = MTSdata.get('Ytrain')           # (IDX x T)
y_train = y_train[:,0:targetsize]         # use only required targets
y_train = torch.from_numpy(y_train)       # convert to tensor (float64)
y_train = y_train.float()                 # convert to float32 - for MSEloss

# get validation feature set (images, 224x224 BGR)
x_val_data = MTSdata.get('Xval')                    # (H x W x C x IDX)
x_val_data = np.transpose(x_val_data,(3,0,1,2))     # move image index to first dimension
x_val = prep_input(x_val_data, trans_x_val)         # transform input to tensor for tvalidation

# get validation target set (dir grad [x, y, z], del pitch, del roll)
y_val = MTSdata.get('Yval')               # (IDX x T)
y_val = y_val[:,0:targetsize]             # use only required targets
y_val = torch.from_numpy(y_val)           # convert to tensor (float64)
y_val = y_val.float()                     # convert to float32 - for MSEloss

# establish training and validation tensor datasets
train_ds = TensorDataset(x_train,y_train)
val_ds = TensorDataset(x_val,y_val)

# estbablish dataloaders for training and validation datasets
train_dl = DataLoader(train_ds,batch_size=batchsize,shuffle=True)
val_dl = DataLoader(val_ds, batch_size=batchsize)

# load CNN to GPU
print('Running on device: {}'.format(device))   # display GPU used
MTSmodel = MTSmodel.to(device)                  # load model to GPU

# initialise training and validation loss records for run
tr_loss_run = np.empty(epochs)      
val_loss_run = np.empty(epochs)

# Fine-tuning process
for epoch in range(epochs):
    print('Epoch {}/{}'.format(epoch+1, epochs))    
    
    # Train
    MTSmodel.train()                    # put model in training mode
    for inputs, targets in train_dl:
        inputs = inputs.to(device)      # load batch of inputs to GPU
        targets = targets.to(device)    # load batch of targets to GPU
        
        pred = MTSmodel(inputs)         # forward propagate - make prediction
        
        tr_loss = loss_func(pred, targets)  # calculate training losses
        tr_loss_run[epoch] += tr_loss       # acculumate training losses for epoch
        
        optimiser.zero_grad()           # zeroise gradients
        tr_loss.backward()              # backpropogation
        optimiser.step()                # update model parameters
    
    # determine Training Loss MSE for epoch    
    tr_loss_run[epoch] = torch.div(tr_loss_run[epoch],len(train_ds))
    
    # Re-jitter x_train for next epoch (disable if not needed)
    #if epoch != epochs:
        #x_train = prep_input(x_trn_data, trans_x_train)    
        
    # Validate
    MTSmodel.eval()                     # put model in evaluation mode
    
    with torch.no_grad():               # disable autograd - no backprop
        for x_val, y_val in val_dl:     # get val batch
            x_val = x_val.to(device)    # load val input to GPU
            y_val = y_val.to(device)    # load val targets to GPU
            val_pred = MTSmodel(x_val)  # predict regression value
            val_loss = loss_func(val_pred, y_val)   # calc val batch loss
            val_loss_run[epoch] += val_loss         # accumulate val loss for epoch
        
        # calc validation loss for epoch
        val_loss_run[epoch] = torch.div(val_loss_run[epoch],len(val_ds))
    
    # update scheduler
    scheduler.step()    # update learning rate scheduler for next epoch
    
    # display epoch losses for training and validation
    print('Training Loss (MSE): {}'.format(tr_loss_run[epoch]))
    print('Validation Loss (MSE): {}'.format(val_loss_run[epoch]))
    print('GPU memory used: {}'.format(torch.cuda.memory_allocated(device)))
    
    # plot losses for each epoch
    epoch_axis = np.linspace(1,epoch+1,epoch+1)
    
    plt.figure(1)
    plt.plot(epoch_axis,tr_loss_run[0:epoch+1], label = "training loss")
    plt.plot(epoch_axis,val_loss_run[0:epoch+1], label = "validation loss")
    plt.xlabel('epochs')
    plt.ylabel('MSE loss')
    plt.legend()
    plt.show()
        
# calculate time to process script
stop = time.time()
proc_time = timedelta(seconds=(stop-start))
print('Process time: {} hr:min:sec'.format(proc_time)) 

# model can be saved using the following:
# torch.save(MTSmodel,'.\MTSmodel#_ep#_bs#_j#_lr#_lrd#_t#_v#.pt')
# CNN_trn_plot_data = {}
# CNN_trn_plot_data['tr_MSEloss'] = tr_loss_run
# CNN_trn_plot_data['val_MSEloss'] = val_loss_run
# scipy.io.savemat('.\CNN_trn_MSEloss_data#.mat', CNN_trn_plot_data)






