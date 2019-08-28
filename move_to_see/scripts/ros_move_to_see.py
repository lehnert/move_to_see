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


import time
import cv2 as cv
import array
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import cProfile
from PIL import Image as I
from pyquaternion import Quaternion as pyQuat
from collections import deque
import scipy
import scipy.sparse.linalg
import scipy.linalg

from ros_interface import ros_interface
from harvey_msgs.msg import moveToSee
import rospy
#import pylab as plb


class move_to_see:

    def __init__(self, number_of_cameras):

        print ('Creating ros interface')

        self.nCameras = number_of_cameras
        self.interface = ros_interface(number_of_cameras)
        #vrep.simxFinish(-1) # just in case, close all opened connections
        #clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

        self.count = 0
        self.counts = []

        self.publisher = rospy.Publisher("/move_to_see_data", moveToSee, queue_size=10)
        self.move_to_see_msg = moveToSee()
        self.move_to_see_msg.nCameras = number_of_cameras
        self.move_to_see_msg.header.stamp = rospy.Time.now()
        self.move_to_see_msg.header.seq = self.count
        self.move_to_see_msg.header.frame_id = "/move_to_see"
        #res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)
        plt.clf()
        fig = plt.figure(1)

        startTime=time.time()

        self.ref_index = 4 #index of middle camera
        self.ref_size = 0 #use this as a stopping condition

        self.plot_array1 = []
        self.max_size = []
        self.max_distance = []
        self.plot_array2 = []

        plt.ion()


        self.camera_positions, self.camera_orientations, self.camera_poses = self.interface.getCameraPositions()

        self.queue_size = 10
        self.delta_magnitude_queue = deque(np.zeros(self.queue_size))
        self.current_delta_mag = 1e5
        self.avg_delta_magnitude = 0.0
        self.tolerance = 0.000001

    def computePixelDifferent(self, image_1,image_2):

        if not (image_1.shape == image_2.shape):
            print ('images are not of the same size')
            return 0

        diffImage = np.zeros(image_1.shape)
        cv.absdiff(image_1,image_2,diffImage)


    def calcNumericalDerivative(self, pixel_size, ref_size, blob_centre,ref_blob_centre):

        dist_weight = 0.0
        size_weight = 1.0
        manip_weight = 0.0

        #ensure weights sum to 1
        dist_weight = dist_weight/(dist_weight+size_weight+manip_weight)
        size_weight = size_weight/(dist_weight+size_weight+manip_weight)
        manip_weight = manip_weight/(dist_weight+size_weight+manip_weight)

        # maxPixels = 256 * 256 * 2/3

        image_cx = 0.5
        image_cy = 0.5

        distance = 1 - math.sqrt(math.pow(blob_centre[0] - image_cx,2) + math.pow(blob_centre[1] - image_cy,2))
        ref_distance = 1 - math.sqrt(math.pow(ref_blob_centre[0] - image_cx,2) + math.pow(ref_blob_centre[1] - image_cy,2))

        distance_diff = (ref_distance - distance) / ref_distance

        norm_pixel_diff = (pixel_size - ref_size) / ref_size #changed this around maybe its wrong

        # manip_diff = manip-ref_manip

        if not (pixel_size == 0.0):
            total_delta = size_weight*norm_pixel_diff + dist_weight*distance_diff #+ manip_weight*manip_diff
        else:
            total_delta = 0.0

        return total_delta

    def getNumericalDerivatives(self):

        delta_matrix = np.zeros([3,3])
        size_matrix = np.zeros([3,3])
        distance_matrix = np.zeros([3,3])

        #get the reference pixel size and manipulability (camera 5 indexed at 0)

        pixel_sizes, blob_centres, pixel_sizes_unfiltered = self.interface.getObjectiveFunctionValues()

        print ("pixel sizes: \n")
        print (np.array(pixel_sizes).reshape((3,3)))
        print "\n"
        # print ("blob_centres: ", blob_centres)
        # print "\n"

        ref_size = pixel_sizes[4]
        ref_blob_centre = blob_centres[4]

        # if(ref_size == 0.0):
        #     return delta_matrix, ref_size

        for i in range(0,9):
            # print("i: ",i)
            # print("blob size: ", pixel_sizes[i])
            # print("blob_centres: ", blob_centres[i])
            # print("manip: ", manip[i])
            delta = self.calcNumericalDerivative(pixel_sizes[i], ref_size, blob_centres[i], ref_blob_centre)
            delta_matrix[np.unravel_index(i, delta_matrix.shape)] = delta

        return delta_matrix, ref_size, pixel_sizes, pixel_sizes_unfiltered

    def showNImages(self, windowName, images):

        row1_stack  = np.hstack( (images[0],images[1],images[2]) )
        row2_stack  = np.hstack( (images[3],images[4],images[5]) )
        row3_stack  = np.hstack( (images[6],images[7],images[8]) )
        full_stack  = np.vstack( (row1_stack,row2_stack,row3_stack) )

        cv.imshow(windowName, full_stack)
        cv.waitKey(1)

    def computeDirDerivative(self, direction_vectors,numerical_derivatives):

        # print ("direction vector shape: ", direction_vectors.shape)
        # print "\n"

        direction_vectors_inv = np.linalg.pinv(direction_vectors.transpose())

        x = numerical_derivatives.reshape((1,len(numerical_derivatives)))

        derivative = x.dot(direction_vectors_inv)


        residuals = derivative.dot(direction_vectors.transpose()) - x


        print ("residuals: \n")
        print (np.insert(residuals,4,0.0).reshape((3,3)))
        print "\n"
        # print ("derivative: ", derivative.shape)

        return derivative

    def weightedAverageVector(self, delta_matrix, camera_positions):

        flat_cost = delta_matrix.reshape([1,9])
        #print "flat_cost: ", flat_cost
        cost_by_pos = flat_cost*camera_positions
        #print "Cost*Pos:",cost_by_pos
        vdes = np.average(cost_by_pos,1)

        return vdes

    #Compute the weighted average of quaternions
    def weighted_avg_quaternions(self, q_array,w):

        i = 0
        Q = np.zeros((4,4))
        w_sum = 0
        # print "w: ",w
        # print "q_array: ", q_array.shape
        for q in q_array:
            # print "q: ",q
            q = scipy.reshape(q, (4,1))
            q_matrix = w[i]*q*q.transpose()
            Q = Q + q_matrix
            w_sum = w_sum + w[i]
            # print "Q: ", Q
            i = i + 1
            # print "  "
            # print "  "

        #compute the eigen values/vectors
        #evals, evecs = np.linalg.eig(Q)
        #find max eigen vector which represents the weighted average of quaternions
        #sorted_evals_evecs = sorted(zip(evals,evecs.T),key=lambda x: x[0].real, reverse=True)
        if(w_sum > 0.0):
            Q = (1.0/w_sum)*Q

        evals,evecs = scipy.linalg.eig(Q)

        #print "Sorted: ", sorted_evals_evecs
        return evecs[0]

    def running_mean(self, x, N):
        cumsum = np.cumsum(np.insert(x, 0, 0))
        return (cumsum[N:] - cumsum[:-N]) / N

    def execute(self):

        print ('Executing servo loop')
        print "\n"
        rate = rospy.Rate(100) # 100hz

        print "Setting arm to initial position"
        if not self.interface.movetoNamedPose("harvey_arm","move_to_see_start", 0.4):
            print "failed to reset robot arm"
            return

        # pose_down = [-0.075,0.0,0.0,-0.05,0.05,0.0,1.0]
        # self.interface.servoPose(pose_down, "harvey_arm", "pi_camera_link", 0.4)


        while not rospy.is_shutdown() and (abs(self.avg_delta_magnitude - self.current_delta_mag) > self.tolerance) and (self.ref_size < 0.15):

            # raw_input("Press Enter to continue...")

            delta_matrix, self.ref_size, pixel_sizes, pixel_sizes_unfiltered = self.getNumericalDerivatives()
            delta_flat = np.delete(delta_matrix.reshape((1,9)),4,None)

            print ('Numerical deltas: \n')
            print (delta_matrix)
            print "\n"

            #delete the fourth camera value as this is the reference camera
            camera_vectors = (np.delete(self.camera_positions,4,1)).transpose()
            camera_vector_mags = np.linalg.norm(camera_vectors,None,1)
            camera_unit_vectors = camera_vectors/camera_vector_mags[:,np.newaxis]

            numerical_derivative = np.divide(delta_flat,camera_vector_mags)
            print ('numerical_derivative (delta/step size): \n')
            print (np.insert(numerical_derivative,4,0.0).reshape((3,3))) #insert 0 back into camera 4 for pretty print
            print "\n"

            self.current_delta_mag = np.sqrt(delta_flat.dot(delta_flat))

            self.delta_magnitude_queue.appendleft(self.current_delta_mag)
            self.delta_magnitude_queue.pop()
            self.avg_delta_magnitude = sum(self.delta_magnitude_queue)/self.queue_size

            print "Current delta magnitude: ", self.current_delta_mag
            print "Average delta magnitude: ", self.avg_delta_magnitude
            print ('Diff of avg delta and current delta ', abs(self.avg_delta_magnitude - self.current_delta_mag))

            # print ('camera_positions: ', self.camera_positions)
            # print "\n"
            # print ('camera_unit_vectors: ', camera_unit_vectors)
            # print "\n"

            derivative = self.computeDirDerivative(camera_unit_vectors,numerical_derivative)
            print ('directional derivative: \n')
            print (derivative)
            print "\n"

            if (np.sum(delta_matrix) == 0.0):
                print "delta matrix = 0"
                break
            else:

                step_size = 0.005

                pose = step_size*derivative
                # pose = [0,0,0.1]
                pose = np.append(pose,0) #add 0 angle for quaternion x
                pose = np.append(pose,0) #add 0 angle for quaternion y
                pose = np.append(pose,0) #add 0 angle for quaternion z
                pose = np.append(pose,1) #add 0 angle for quaternion w

                pose = pose.reshape((7,))
                # print ('pose: \n'
                # print (pose)
                # print "\n"

                self.interface.servoPose(pose, "harvey_pi_camera", "pi_camera_link", 0.2)

                self.counts.append(self.count)
                self.count = self.count + 1

                self.move_to_see_msg.header.stamp = rospy.Time.now()
                self.move_to_see_msg.header.seq = self.count
                self.move_to_see_msg.header.frame_id = "/move_to_see"
                self.move_to_see_msg.pixel_sizes = pixel_sizes
                self.move_to_see_msg.pixel_sizes_unfiltered = pixel_sizes_unfiltered

                print ("pixel sizes: \n")
                print (np.array(pixel_sizes).reshape((3,3)))
                print "\n"
                self.move_to_see_msg.ref_pixel_size = self.ref_size

                self.publisher.publish(self.move_to_see_msg)

                #end control loop by maintaining desired rate
                # time.sleep(0.05)
                rate.sleep()


            print ('Current object reference size: ', self.ref_size)
            print "\n"

            print ('Diff of avg delta and current delta ', abs(self.avg_delta_magnitude - self.current_delta_mag))
            print ('should be smaller then tolerance: ', self.tolerance)
            print "\n"
        else:
            print ('Failed connecting to remote API server')

        print ('cost within tolerance, finished')

if __name__=="__main__":

    rospy.init_node("harvey_move_to_see")

    print "Running camera array master"

    mvs = move_to_see(9)
    mvs.execute()
    #cProfile.run('move_to_see()')
