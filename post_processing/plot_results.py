# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 15:43:33 2018

@author: chris
"""

import pickle
import time

import matplotlib.pylot as plt


def plot_results(data):

    plt.clf()
    fig = plt.figure()
        
    plt.ion()
    plt.show()
    plt.hold(True)
    
    plt.figure(1)
    self.fig.clf()
                        
    #plt.subplot(211)
    plt.plot(self.counts,self.ref_pixel_sizes,'r')
                    
    #plt.subplot(212)
    plt.plot(self.counts,self.ref_manips,'b')
                    
    self.fig.canvas.draw()
    self.fig.canvas.flush_events()



if __name__=="__main__":
    
    
    file_name = "vrep_sim_2018_09_12-15_21_45.pickle"
    fobject = open(file_name,'r')
    
    data = picklel.load(fobject)
    
    plot_results(data)

