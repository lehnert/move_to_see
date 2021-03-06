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
#import array
import numpy as np
import matplotlib.pyplot as plt
import math
import collections
#import cProfile
#from PIL import Image as I
#from pyquaternion import Quaternion as pyQuat
from collections import deque
import scipy
import scipy.sparse.linalg
import scipy.linalg
from geometry_msgs.msg import PoseStamped

import rospy
from tf.transformations import quaternion_from_euler

import tf2_geometry_msgs

#import pylab as plb
    
class Xtype:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)
            

class move_to_see:

    def __init__(self, number_of_cameras, interface,step_size=0.1, size_weight=0.8,manip_weight=0.2,end_tolerance=1e-6, max_pixel=15 ):

        
        self.nCameras = number_of_cameras

        if interface == "ROS":
            print ('Creating ros interface')
            import ros_interface_naive as ri
            self.interface = ri.ros_interface(number_of_cameras)
        elif interface == "VREP":
            print ('Creating vrep interface')
            import vrep_interface as vi
            self.interface = vi.vrep_interface(number_of_cameras)
            self.interface.start_sim()
        #vrep.simxFinish(-1) # just in case, close all opened connections
        #clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

        self.set_orientation = True
        
        self.noise_std = 0.001

        self.step_size = step_size

        self.count = 0
        self.counts = []

        #res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)
        if self.interface.type == "VREP":
            plt.clf()
            self.fig = plt.figure(1)
            
            plt.ion()
            plt.show()
            
        self.dist_weight = 0.0
        self.size_weight = size_weight
        self.manip_weight = manip_weight

        if self.interface.type == "VREP":
            self.image_width = 256
            self.image_height = 256
            self.image_cx = 0.5
            self.image_cy = 0.5
                    
            self.FOV_x = 60
            self.FOV_y = 60
        elif self.interface.type == "ROS":
            self.image_width = 640
            self.image_height = 480
            self.image_cx = 320
            self.image_cy = 240
            self.FOV_x = 62.2
            self.FOV_y = 48.8
        
        


        startTime=time.time()

        self.ref_index = 4 #index of middle camera
        self.ref_size = 0 #use this as a stopping condition

        self.max_size = []
        self.max_distance = []
       
        #self.camera_positions, self.camera_orientations, self.camera_poses = self.interface.getCameraPositions()

        self.queue_size = 5
        self.tolerance = end_tolerance
        self.max_pixel = max_pixel
    
        self.ee_poses = []
        self.pose_deltas = []
        self.ref_size = 0.0
        self.count = 0
        self.ref_pixel_sizes = []
        self.ref_pixel_sizes_zero_noise = []
        self.ref_manips = []
        self.counts = []
        self.accum_step_size = 0.0
        self.gradients = []
        self.avg_abs_gradient_queue = deque(1e5*np.ones(self.queue_size))
        self.abs_gradient = 1e5
        self.avg_abs_gradient = 1e5
        
    def reset(self):
        
        self.ee_poses = []
        self.pose_deltas = []
        self.ref_size = 0.0
        self.count = 0
        self.ref_pixel_sizes = []
        self.ref_pixel_sizes_zero_noise = []
        self.ref_manips = []
        self.counts = []
        self.accum_step_size = 0.0
        self.gradients = []
        self.avg_abs_gradient_queue = deque(1e5*np.ones(self.queue_size))
        self.abs_gradient = 1e5
        self.avg_abs_gradient = 1e5


    def setObjectiveFunctionWeights(self,size_weight,manip_weight):
        self.size_weight = size_weight
        self.manip_weight = manip_weight
        
    def setCameraPosition(self,radius,link_offset,camera_angle,set_euler_angles):
        
        self.interface.setCameraOffsets(radius,link_offset,camera_angle,set_euler_angles)
        
        self.camera_positions, self.camera_orientations, self.camera_poses = self.interface.getCameraPositions()
        
        #delete the fourth camera value as this is the reference camera
        self.camera_vectors = (np.delete(self.camera_positions,4,1)).transpose()
        self.camera_vector_mags = np.linalg.norm(self.camera_vectors,None,1)
        self.camera_unit_vectors = self.camera_vectors/self.camera_vector_mags[:,np.newaxis]
        
    def initCameraPosition(self):
        
        self.camera_positions, self.camera_orientations, self.camera_poses = self.interface.getCameraPositions()
        self.camera_vectors = (np.delete(self.camera_positions,4,1)).transpose()
        self.camera_vector_mags = np.linalg.norm(self.camera_vectors,None,1)
        self.camera_unit_vectors = self.camera_vectors/self.camera_vector_mags[:,np.newaxis]
        

    def computePixelDifferent(self, image_1,image_2):

        if not (image_1.shape == image_2.shape):
            print ('images are not of the same size')
            return 0

        diffImage = np.zeros(image_1.shape)
        cv.absdiff(image_1,image_2,diffImage)
            
    

    def calcNumericalDerivative(self, x1, x2):

       # ref_size = x1.pixel_size
        #ref_blob_centre = x1.blob_centre
        #ref_manip = x1.manip
        dist_weight = self.dist_weight
        size_weight = self.size_weight
        manip_weight = self.manip_weight
        
        #ref_size, blob_centre, 
    
        #ensure weights sum to 1
        dist_weight = dist_weight/(dist_weight+size_weight+manip_weight)
        size_weight = size_weight/(dist_weight+size_weight+manip_weight)
        manip_weight = manip_weight/(dist_weight+size_weight+manip_weight)

        # maxPixels = 256 * 256 * 2/3

        #image_cx = 0.5
        #image_cy = 0.5
              
        
        #distance1 = 1 - math.sqrt(math.pow(x1.blob_centre[0] - self.image_cx,2) + math.pow(x1.blob_centre[1] - self.image_cy,2))
        #distance2 = 1 - math.sqrt(math.pow(x2.blob_centre[0] - self.image_cx,2) + math.pow(x2.blob_centre[1] - self.image_cy,2))
        
        
        #if ref_distance > 0:
             # manip_diff = manip-ref_manip

        if (x2.pixel_size > 0.0) and (x1.pixel_size > 0.0):
            #distance_diff = (distance1 - distance2) / distance1
            
            norm_pixel_diff = (x2.pixel_size - x1.pixel_size) / x1.pixel_size #changed this around maybe its wrong
            
            if self.interface.type == "ROS":
                manip_diff = 0
            else:
                manip_diff = (x2.manip - x1.manip) / x1.manip
            
            total_delta = size_weight*norm_pixel_diff + manip_weight*manip_diff
            ref_objective_value = size_weight*x1.pixel_size + manip_weight*x1.manip
        else:
            total_delta = 0.0
            ref_objective_value = 0.0

        return total_delta, ref_objective_value

    def getNumericalDerivatives(self,noise_std=0.001,use_noise=False):

        delta_matrix = np.zeros([3,3])

        #get the reference pixel size and manipulability (camera 5 indexed at 0)
        if self.interface.type == "VREP":
            pixel_sizes, blob_centres, manip = self.interface.getObjectiveFunctionValues()
            pixel_sizes_noise = pixel_sizes + np.random.normal(0,noise_std,len(pixel_sizes))
            
        elif self.interface.type == "ROS":
            pixel_sizes, blob_centres, pixel_sizes_unfiltered = self.interface.getObjectiveFunctionValues()
                  
        x = []
        x_ref = Xtype(pixel_size=0.0,blob_centre=[0.0,0.0], manip=0.0)
            
        for i in range(0,self.nCameras):

            
            x.append(Xtype(pixel_size=0.0,blob_centre=[0.0,0.0], manip = 0.0))           
            
            
            
            if not blob_centres[i] == [] and not blob_centres[4] == [] and not pixel_sizes[i] == [] and not pixel_sizes[4] == []:
                x_ref.blob_centre = blob_centres[4]
                x[i].blob_centre = blob_centres[i]
            
                
                x_ref.manip = 0.0
                x[i].manip = 0.0
                
                x_ref.pixel_size = pixel_sizes[4]
                x[i].pixel_size = pixel_sizes[i]            
                
                
                
            delta,x_ref_obj_val = self.calcNumericalDerivative(x_ref, x[i])
            
           
            delta_matrix[np.unravel_index(i, delta_matrix.shape)] = delta

        return delta_matrix, x_ref, x, pixel_sizes[4], x_ref_obj_val

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


        #print ("residuals: \n")
        #print (np.insert(residuals,4,0.0).reshape((3,3)))
        #print "\n"
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
    
    def compute_roll_pitch(self, blob_centre):
        
        if self.interface.type == "VREP":
            image_width = 1
            image_height = 1
            image_cx = 0.5
            image_cy = 0.5
        elif self.interface.type == "ROS":
            image_width = 640
            image_height = 480
            image_cx = 320
            image_cy = 240
        
        delta_x = (blob_centre[0] - self.image_cx)/image_width
        delta_y = (blob_centre[1] - self.image_cy)/image_height
        
        #print ("Blob centre x = ", blob_centre[0])
        #print ("Blob centre y = ", blob_centre[1])
        #print ("delta_x = ", delta_x)
        #print ("delta_x = ", delta_y)
        
        
        
        dRoll = math.radians(self.FOV_x*delta_x)
        dPitch =  math.radians(self.FOV_y*delta_y)
        
        if abs(dPitch) < 0.02:
            dPitch= 0.0
        if abs(dRoll) < 0.02:
            dRoll = 0.0
        
        return dRoll,dPitch
    
    
    def executeNaive(self):
        
        
        self.interface.getScene()
        start_image = self.interface.bridge.imgmsg_to_cv2(self.interface.scene.image, desired_encoding="passthrough")
        
        
        ee_pose1 = self.interface.getCurrentPoseNaive()
        
        pixel_sizes1 = self.interface.getPixelSize()
        print "Pixel size 1: ", pixel_sizes1
        target_pose  = self.interface.getCapsicumDepth()
        
        #pixel_sizes1, blob_centres1, manip1 = self.interface.getObjectiveFunctionValues()
        
        #target_pose = self.interface.getDepthPoint()
        
        print "Target pose: ", target_pose
               
        
        #target_pose[2] = target_pose[2] - 0.15
        
        
        poseStamp = PoseStamped()

            
        poseStamp.pose.position.x = target_pose.x
        poseStamp.pose.position.y = target_pose.y
        poseStamp.pose.position.z = target_pose.z
        
        
        poseStamp.pose.orientation.x = 0.0
        poseStamp.pose.orientation.y = 0.0
        poseStamp.pose.orientation.z = 0.0
        poseStamp.pose.orientation.w = 1.0
        
        
        self.interface.tf_listener.waitForTransform("camera_link","harvey_base_link", rospy.Time(0), rospy.Duration(5))
        poseStamp.header.frame_id = "harvey_base_link"
        poseStamp.header.stamp = rospy.Time(0)
        transformed_pose = self.interface.tf_listener.transformPose("camera_link", poseStamp)
        
        transformed_pose.pose.position.x = transformed_pose.pose.position.x - 0.1
        
        transformed_pose.pose.orientation.x = 0.0
        transformed_pose.pose.orientation.y = 0.0
        transformed_pose.pose.orientation.z = 0.0
        transformed_pose.pose.orientation.w = 1.0
        
        self.interface.servoPoseNaive(transformed_pose.pose)
        
        #blob_centres2, manip2 = self.interface.getObjectiveFunctionValues()
        pixel_sizes2 = self.interface.getPixelSize()
        print "Pixel size 2: ", pixel_sizes1
        
        ee_pose2 = self.interface.getCurrentPoseNaive()
               
       
        #end_image = self.interface.getImage()
        self.interface.getScene()
        end_image = self.interface.bridge.imgmsg_to_cv2(self.interface.scene.image, desired_encoding="passthrough")
        
        ret_dict = {}

        ret_dict['ref_pixel_size'] = [pixel_sizes1, pixel_sizes2]
        ret_dict['ee_pose'] = [ee_pose1, ee_pose2]
        #ret_dict['pose_deltas'] = [transformed_pose.pose]
        ret_dict['start_image'] = [start_image]
        ret_dict['end_image'] = [end_image]
        
        #distance_to_occluder = target_pose[2]
               
        #while ref_pixel_size < self.max_pixel and not ref_pixel_size == 0.0 and distance_to_occluder > 0.15:
        
            #pixel_sizes, blob_centres, manip = self.interface.getObjectiveFunctionValues()
            #ref_pixel_size = pixel_sizes[4]
            
            #target_pose = self.interface.getDepthPoint()
            #distance_to_occluder = target_pose[2]
            #target_pose[2] = 0.025
            
            #servo_pose = np.zeros((6,))
            
            #servo_pose[0] = target_pose[0]
            #servo_pose[1] = target_pose[1]
            #servo_pose[2] = target_pose[2]
                      
            
            #print "ref_pixel_size", ref_pixel_size
           # print "distance_to_occluder", distance_to_occluder
        
            #self.interface.servoPoseEuler(servo_pose)
        
        
        print "Finished naive method", pixel_sizes2
        
        return ret_dict
    

    def execute(self):

        print ('Executing servo loop')
        print "\n"
        
        use_noise=False
        
        if self.interface.type == "ROS":
            use_noise = False
            rate = rospy.Rate(0.5) # 100hz
        


        # pose_down = [-0.075,0.0,0.0,-0.05,0.05,0.0,1.0]
        # self.interface.servoPose(pose_down, "harvey_arm", "pi_camera_link", 0.4)

        print ('Diff of avg delta and current delta ', self.avg_abs_gradient)
        print ('should be smaller then tolerance: ', self.tolerance)
        
        
        if self.interface.type == "VREP":
            self.start_image = self.interface.getImage()
        
        delta_matrix, self.x_ref, self.x, self.ref_size_no_noise, self.x_ref_obj_val = self.getNumericalDerivatives(self.noise_std,use_noise)
        
        while (self.avg_abs_gradient > self.tolerance) and (self.x_ref.pixel_size < self.max_pixel) and (self.count < 200):

            # raw_input("Press Enter to continue...")

            delta_matrix, self.x_ref, self.x, self.ref_size_no_noise, self.x_ref_obj_val = self.getNumericalDerivatives(self.noise_std,use_noise)

            delta_flat = np.delete(delta_matrix.reshape((1,9)),4,None)

            #print ('Numerical deltas: \n')
            #print (delta_matrix)
            #print "\n"
            
            if (np.sum(delta_matrix) == 0.0):
                print "delta matrix = 0, stopping"
                break
            else:
   
                numerical_derivative = np.divide(delta_flat,self.camera_vector_mags)
                    
                self.gradient = self.computeDirDerivative(self.camera_unit_vectors,numerical_derivative)
                self.gradients.append(self.gradient)
                
                dRoll, dPitch = self.compute_roll_pitch(self.x_ref.blob_centre)
                
                #print "Roll = ", dRoll
                #print "Pitch = ", dPitch
                
                #print ("Roll angle: ", math.degrees(dRoll))
                #print ("Roll pitch: ", math.degrees(dPitch))
                
                #self.current_delta_mag = np.sqrt(delta_flat.dot(delta_flat))
                self.avg_abs_gradient = sum(self.avg_abs_gradient_queue)/self.queue_size
                self.abs_gradient = np.linalg.norm(self.gradient)
                self.avg_abs_gradient_queue.appendleft(self.abs_gradient)
                self.avg_abs_gradient_queue.pop()
                
                pose_delta = self.step_size*self.gradient
                
                ##################################################
                
                if self.interface.type == "ROS":
                    # pose = [0,0,0.1]
                    #q = quaternion_from_euler(0,0,0)
                    q = quaternion_from_euler(-dRoll/2,-dPitch/2,0)
                    #q = quaternion_from_euler(-dRoll,0,0)
                    
                    #pose_delta[0,0] = 0
                    #pose_delta[0,1] = 0
                    #pose_delta[0,2] = 0
                    
                    pose_delta = np.append(pose_delta,q[0]) #add quaternion x
                    pose_delta = np.append(pose_delta,q[1]) #add quaternion y
                    pose_delta = np.append(pose_delta,q[2]) #add quaternion z
                    pose_delta = np.append(pose_delta,q[3]) #add quaternion w
                    pose_delta = pose_delta.reshape((7,))
                    
                    
                    #print "Pose delta = ", pose_delta
                    self.interface.servoPose(pose_delta)
                    
                    ee_pose = self.interface.getCurrentPose()
                    self.ee_poses.append(ee_pose)
                    
                    #self.interface.publish_data(pixel_sizes,pixel_sizes_unfiltered,self.x_ref.pixel_size, self.count)                   
                
                elif self.interface.type == "VREP":
                   
                    #pose = [0,0,0]
                    if self.set_orientation==True:
                        pose_delta = np.append(pose_delta,-dPitch/2) #add 0 angle for euler x
                        pose_delta = np.append(pose_delta,-dRoll/2)
                    else:
                        pose_delta = np.append(pose_delta,0) 
                        pose_delta = np.append(pose_delta,0) 
                    
                    pose_delta = np.append(pose_delta,0)
                    pose_delta = pose_delta.reshape((6,))

                    # apply pose to robot in VREP
                    self.interface.servoPoseEuler(pose_delta)

                    #add data to lists
                    ee_pose = self.interface.getCurrentPose()
                    self.ee_poses.append(ee_pose)
                    
                ######################################3    
                    
                self.pose_deltas.append(pose_delta)
                    
                self.ref_pixel_sizes.append(self.x_ref.pixel_size)
                self.ref_manips.append(self.x_ref.manip)
                    
                self.count = self.count + 1
                self.counts.append(self.count)
                    
                    #self.ref_size_data.append(self.x_ref.pixel_size)
                    #self.ref_size_data2.append(self.ref_size_no_noise)
                    
                # plot runtime data
                if self.interface.type == "VREP":
                    plt.figure(1)
                    self.fig.clf()
                        
                    plt.subplot(211)
                    plt.plot(self.counts,self.ref_pixel_sizes,'r')
                    
                    plt.subplot(212)
                    plt.plot(self.counts,self.ref_manips,'b')
                    
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    #plt.draw()
                    

                #end control loop by maintaining desired rate
                if self.interface.type == "ROS":
                    rate.sleep()
    
                #print ('directional derivative: \n')
                #print (self.gradient)
                #print "\n"


            print ('Avg Abs Gradient: ', self.avg_abs_gradient)
            print ('will stop if smaller then tolerance: ', self.tolerance)
            print ('Ref pixel size is ', self.x_ref.pixel_size)
            print ('Will terminate when greater than max pixel size ', self.max_pixel)
            print "\n"
            #print ('numerical_derivative (delta/step size): \n')
            #insert 0 back into camera 4 for pretty print
            #print (np.insert(numerical_derivative,4,0.0).reshape((3,3)))
            #print "\n"
    
        if self.interface.type == "VREP":
            self.end_image = self.interface.getImage()
        
        print ('cost within tolerance, finished')
        
        
        print ('Avg Abs Gradient: ', self.avg_abs_gradient)
        print ('stopped if smaller then tolerance: ', self.tolerance)
        print ('Ref pixel size is ', self.x_ref.pixel_size)
        print ('Will terminate when greater than max pixel size ', self.max_pixel)
        print "\n"
        
        ret_dict = {}

        ret_dict['count'] = self.counts
        ret_dict['gradient'] = self.gradients
        ret_dict['ref_pixel_size'] = self.ref_pixel_sizes
        ret_dict['ref_manip'] = self.ref_manips
        ret_dict['ee_pose'] = self.ee_poses
        ret_dict['pose_deltas'] = self.pose_deltas
        
        if self.interface.type == "VREP":
            ret_dict['start_end_images'] = [self.start_image, self.end_image]
        
        return ret_dict

if __name__=="__main__":
    

    interface = "ROS"

    if interface == "ROS":
        rospy.init_node("harvey_move_to_see")
        print "Running move to see node"

        mvs = move_to_see(9,"ROS",step_size=0.001, size_weight=1.0, manip_weight=0.0,end_tolerance=0.75,max_pixel=0.4)
        mvs.initCameraPosition()

    else:
        mvs = move_to_see(9,"VREP")

    data = mvs.execute()
    #cProfile.run('move_to_see()')
