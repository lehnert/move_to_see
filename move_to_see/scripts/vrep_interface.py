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
import numpy as np
#import cv2 as cv
#import array
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#import math
#import cProfile
#from PIL import Image as I
#from pyquaternion import Quaternion as pyQuat
#from collections import deque
#import scipy
#import scipy.sparse.linalg
#import scipy.linalg
#import pylab as plb


try:
    import vrep
except:
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


class vrep_interface:

    def __init__(self, number_of_cameras, ip='127.0.0.1', port=19997):
        
        self.type = "VREP"

        # setup VREP connection
        print ('connecting to vrep')
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientID=vrep.simxStart(ip,port,True,True,5000,1) # Connect to V-REP
        
        res,self.ee_handle = vrep.simxGetObjectHandle(self.clientID,'camera_link',vrep.simx_opmode_oneshot_wait)
        res,self.base_frame_handle = vrep.simxGetObjectHandle(self.clientID,'base_frame',vrep.simx_opmode_oneshot_wait)
        res,self.ref_camera_color_handle = vrep.simxGetObjectHandle(self.clientID,'Vision_sensor_ref4',vrep.simx_opmode_oneshot_wait)
        
        self.camera_handles = []
        for index in range(number_of_cameras):
            res,camera_handle = vrep.simxGetObjectHandle(self.clientID,'Vision_sensor_ref'+str(index),vrep.simx_opmode_oneshot_wait)
            self.camera_handles.append(camera_handle)
        
        #self.clientID=vrep.simxStart('192.168.1.39',19997,True,True,5000,5) # Connect to V-REP

        # ensure simulation has been restarted
        vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
        time.sleep(1)

        if self.clientID!=-1:
            print ('Connected to remote API server')
            # Now send some data to V-REP in a non-blocking fashion:
            vrep.simxAddStatusbarMessage(self.clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)

        self.nCameras = number_of_cameras

        # internal memory for pixel info
        self.pixel_sizes = self.init_list_of_objects(number_of_cameras, 0)
        self.pixel_sizes_filtered = self.init_list_of_objects(number_of_cameras, 0)
        self.blob_centres = self.init_list_of_objects(number_of_cameras, 0)

    def __del__(self):
        vrep.simxFinish(self.clientID)
        
    # OLD but left here for use in previous versions 
    def getImage(self):
        res,resolution,image = vrep.simxGetVisionSensorImage(self.clientID,self.ref_camera_color_handle,0,vrep.simx_opmode_oneshot_wait)
        
        img = np.array(image,dtype=np.uint8)
        ret = img.reshape([resolution[0],resolution[1],3])
        
        #returns image in (HWC)
        return ret[...,::-1]

    def getRefImage(self):
        res,resolution,image = vrep.simxGetVisionSensorImage(self.clientID,self.ref_camera_color_handle,0,vrep.simx_opmode_oneshot_wait)
        
        img = np.array(image,dtype=np.uint8)
        ret = img.reshape([resolution[0],resolution[1],3])
        
        return ret[...,::-1]

    def getImage(self,camera_index):
        res,resolution,image = vrep.simxGetVisionSensorImage(self.clientID,self.camera_handles[camera_index],0,vrep.simx_opmode_oneshot_wait)
        
        img = np.array(image,dtype=np.uint8)
        ret = img.reshape([resolution[0],resolution[1],3])
        
        return ret[...,::-1]

    def init_joints(self):
        # UR5
        #return self.set_joints_degrees(np.array([0.0,-360,-270-45,-90,-135,-90,90])) # normal
        #return self.set_joints_degrees(np.array([0,-130,-45,90,0,190,0])) # 3 7 5 idx out of range mts l301
        #return self.set_joints_degrees(np.array([0,-30,30,-30,0,120,0])) # 3 9 5 nonetype jointvals mts l324 / idx out of range mts l301
        #return self.set_joints_degrees(np.array([0,90,-30,55,0,0,0])) # 3 9 5 servo fail MvFuncs l594 / idx out of range mts l301
        #return self.set_joints_degrees(np.array([0,0,0,0,0,90,0])) # 3 9 6
        #return self.set_joints_degrees(np.array([0,90,0,0,0,0,0])) # 3 10 6
        #return self.set_joints_degrees(np.array([0.0,-20,-270-45,-70,-135,-90,90])) # 4 6 7

        # Panda
        #return self.set_joints_degrees(np.array([0,0,0,-90,-90,90,0])) # normal
        return self.set_joints_degrees(np.array([0,0,0,-45,-90,90,0])) # normal

    def initialise_sim(self,ee_pose,frame=[]):
        inFloats = ee_pose
        inStrings = [frame]
        emptyBuff = bytearray()        
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'initialise',[],inFloats,inStrings,emptyBuff,vrep.simx_opmode_oneshot_wait)

    def start_sim(self):
        print ("starting sim")
        vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait) # Start the simulation to V-REP
        
    def stop_sim(self):
        print ("stopping sim")
        vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait) # ensure simulation has been restarted

    def disconnect_vrep():
        print("Closing connection to vrep")
        vrep.simxFinish(-1)

    def init_list_of_objects(self,size1, size2):
        list_of_objects = list()
        for i in range(0,size1):
            list_of_objects.append( list() )
        if size2 > 0:
            for i in range(0,size1):
                for j in range(0,size2):
                    # print list_of_objects
                    list_of_objects[i].append(0.0)
        return list_of_objects
    
    def set_joints_degrees(self,joint_values):
        inFloats = np.deg2rad(joint_values)
        inFloats[0] = joint_values[0]
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setJoints',[],inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        return res

    def set_object_position(self,object_name,position,orientation,ref_name):
        res,handle = vrep.simxGetObjectHandle(self.clientID,object_name,vrep.simx_opmode_oneshot_wait)
        
        if ref_name == -1:
            ref_handle = -1
        else:
            res,ref_handle = vrep.simxGetObjectHandle(self.clientID,ref_name,vrep.simx_opmode_oneshot_wait)
        
        if not position == []:       
            vrep.simxSetObjectPosition(self.clientID,handle,ref_handle,position,vrep.simx_opmode_oneshot_wait)
        if not orientation == []:
            # print orientation
            vrep.simxSetObjectOrientation(self.clientID,handle,ref_handle,orientation,vrep.simx_opmode_oneshot_wait)

    def setCameraOffsets(self, radius, link_offset, angle, set_euler_angles=False):
        if set_euler_angles:
            inInts=[1]
        else:
            inInts=[0]

        inFloats=[radius,link_offset,angle,angle]
        emptyBuff = bytearray()

        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setCameraOffsets',[],inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    def getDepthPoint(self):
        emptyBuff = bytearray()

        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'getReferenceDepthPoint',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

        depthPoint = np.zeros((3,))

        depthPoint[0] = retFloats[0]
        depthPoint[1] = retFloats[1]
        depthPoint[2] = retFloats[2]
        
        #print("Depth Point = ", depthPoint)
        
        return depthPoint
    
    def getCurrentPose(self):
        emptyBuff = bytearray()
        res, position = vrep.simxGetObjectPosition(self.clientID,self.ee_handle,self.base_frame_handle,vrep.simx_opmode_oneshot_wait)
        res, orientation = vrep.simxGetObjectOrientation(self.clientID,self.ee_handle,self.base_frame_handle,vrep.simx_opmode_oneshot_wait)
        #res, transform = vrep.simxGetObjectMatrix(self.clientID,self.ee_handle,self.base_frame_handle,vrep.simx_opmode_oneshot_wait)
        return position+orientation
    
    def getCameraPositions(self):
        inInts=[]
        inFloats=[]
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'getCameraOffsets',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        
        camera_positions = np.zeros([3,self.nCameras])
        camera_poses = np.zeros([5,self.nCameras])
        camera_orientations = []
        
        stride = 6
        
        #print(retFloats)
            
        #print retFloats
        if len(retFloats) > 0:
            for i in range(0,self.nCameras):
                #print i
                camera_orientations.append(np.zeros((4,)))
    
                camera_positions[0,i] = retFloats[(i*stride)]
                camera_positions[1,i] = retFloats[(i*stride)+1]
                camera_positions[2,i] = retFloats[(i*stride)+2]
    
                camera_poses[0,i] = retFloats[(i*stride)]
                camera_poses[1,i] = retFloats[(i*stride)+1]
                camera_poses[2,i] = retFloats[(i*stride)+2]
                camera_poses[3,i] = retFloats[(i*stride)+3]
                camera_poses[4,i] = retFloats[(i*stride)+4]
                # camera_poses[5,i] = retFloats[(i*7)+5]
                # camera_poses[6,i] = retFloats[(i*7)+6]
    
                # camera_orientations[0,i] = retFloats[(i*7)]
                # camera_orientations[1,i] = retFloats[(i*7)+1]
                # camera_orientations[2,i] = retFloats[(i*7)+2]
                # camera_orientations[3,i] = retFloats[(i*7)+3]
    
                camera_orientations[i][0] = retFloats[(i*stride)+3]
                camera_orientations[i][1] = retFloats[(i*stride)+4]
                camera_orientations[i][2] = retFloats[(i*stride)+5]
                # camera_orientations[i][3] = retFloats[(i*7)+6]
        else:
            print ("Error no camera values returned")

        return camera_positions,np.array(camera_orientations), camera_poses

    def servoTargetCamera(self, target_camera, step_size):
        inInts = target_camera
        inFloats = [step_size]
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movetoCamera',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        # print "retInts: ", retInts
        if(retInts[0] == 1):
            print ('servoTargetCamera: servoing failed')
            return False
        else:
            return True

    def getObjectiveFunctionValues(self):
        inInts = []
        inFloats = []
        emptyBuff = bytearray()
        t1 = time.time()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,
                                                                               'MoveFunctions',
                                                                               vrep.sim_scripttype_childscript,
                                                                               'getCostMatrix',
                                                                               [],[],[],emptyBuff,
                                                                               vrep.simx_opmode_oneshot_wait)

        t2 = time.time()
        print("Time to get objective function values = ", t2-t1)
        
        delta_matrix = np.zeros([3,3])
        size_matrix = np.zeros([3,3])
        distance_matrix = np.zeros([3,3])
        manip_matrix = np.zeros([3,3])

        # print retFloats
        pixel_size = []
        blob_centre = []
        manip = []
        if res == 0:
            for i in range(0,self.nCameras):
                pixel_size.append(retFloats[(i*4)])
                blob_centre.append([retFloats[(i*4)+1], retFloats[(i*4)+2]])
                manip.append(retFloats[(i*4)+3])
            return True, pixel_size, blob_centre, manip
        else:
            print("Error calling get cost sim function, ret code: ", res )
            return False, pixel_size, blob_centre, manip#0,0,0 # This caused errors, check if it's ok now..
        
        
        #get the reference pixel size and manipulability (camera 5 indexed at 0)
        #ref_size = pixel_size[4]
        #ref_manip = manip[4]
        #ref_blob_centre = blob_centre[4]

        #return pixel values and blob centres (last retval is for noisy data)       
        

        ####################
        # previous vrep code
        # if(ref_size == 0.0):
        #     return delta_matrix, distance_matrix, manip_matrix
        #
        # for i in range(0,9):
        #     # print("i: ",i)
        #     # print("blob size: ", pixel_size[i])
        #     # print("blob_centre: ", blob_centre[i])
        #     # print("manip: ", manip[i])
        #     delta,norm_distance = deltaFunction(pixel_size[i], ref_size, blob_centre[i], ref_blob_centre, manip[i], ref_manip)
        #     delta_matrix[np.unravel_index(i, delta_matrix.shape)] = delta
        #     # size_matrix[np.unravel_index(i,size_matrix.shape)] = norm_size
        #     distance_matrix[np.unravel_index(i, distance_matrix.shape)] = norm_distance
        #     manip_matrix[np.unravel_index(i, manip_matrix.shape)] = manip[i]
        #
        # return delta_matrix, distance_matrix, manip_matrix, ref_size
        #####################

    def servoXYZ(self, xyz_delta):

        inInts=[]
        inFloats=xyz_delta
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'moveXYZdelta',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        if(retInts[0] == 1):
            print ('servoXYZ: servoing failed')
            return False
        else:
            return True

    def setXYZ(self, position):

        inInts=[]
        inFloats=position
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'moveXYZ',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        if(retInts[0] == 1):
            print ('setXYZ: servoing failed')
            return False
        else:
            return True

    def setPose(self, pose):

        inInts=[]
        inFloats= pose
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setPose',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        # print "retInts: ", retInts
        if(retInts[0] == 0):
            return True
        else:
            print ('setPose: servoing failed')
            return False

    def servoPose(self, pose):

        inInts=[]
        inFloats= pose
        emptyBuff = bytearray()
        time.no
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movePose',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        # print "retInts: ", retInts
        if(retInts[0] == 0):
            return True
        else:
            print ('servoPose: servoing failed')
            return False
        # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

        # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    def servoPoseEuler(self, pose):

        inInts=[]
        inFloats= pose
        emptyBuff = bytearray()
        t1 = time.time()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movePoseEuler',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        t2 = time.time()

        if res > 0:
            print('error calling servo pose')
            return False
        #print("Time to servo pose = ", t2-t1)
        # print "retInts: ", retInts
        
        else:
            if(retInts[0] == 0):
                return True
            elif (retInts[0] == 1):
                print ('servoPoseEuler: servoing failed')
                return False
            elif (retInts[0] == 2):
                print ('servoPoseEuler: servoing not performed')
                return False
        # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

        # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    def setPoseEuler(self, pose):

        inInts=[]
        inFloats= pose
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setPoseEuler',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        # print "retInts: ", retInts
        if(retInts[0] == 0):
            return True
        else:
            print ('setPoseEuler: servoing failed')
            return False
        # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

        # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    # get current joint values
    def getJointParameters(self):
        emptyBuffer = bytearray()
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction( self.clientID,
                                                                                  'MoveFunctions',
                                                                                  vrep.sim_scripttype_childscript,
                                                                                  'getJointValues',
                                                                                  [],[],[],emptyBuffer,
                                                                                  vrep.simx_opmode_oneshot_wait)
        if res == vrep.simx_return_ok:
            #print (retFloats)
            return retFloats
        else:
            print ('could not get joint values')
