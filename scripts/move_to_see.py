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



try:
    import vrep
except:
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

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
#import pylab as plb


def computePixelDifferent(image_1,image_2):

    if not (image_1.shape == image_2.shape):
        print ('images are not of the same size')
        return 0

    diffImage = np.zeros(image_1.shape)
    cv.absdiff(image_1,image_2,diffImage)

def detectRed(imageRGB,red_threshold):

    hsv = cv.cvtColor(imageRGB,cv.COLOR_RGB2HSV)


    # ret,binary_image = cv.threshold(imageRGB[:,:,0],red_threshold,255,cv.THRESH_BINARY_INV)
    #binary_image   = np.zeros( (imageRGB.shape[0],imageRGB.shape[1])).astype('uint8')
    #binary_image[imageRGB[:,:,0]>red_threshold/255.] = 1;
    #binary_image[imageRGB[:,:,1]>120/255.] = 0
    #binary_image[imageRGB[:,:,2]>120/255.] = 0

    #hsv[:,:,0] = (hsv[:,:,0] + 45) % 180

    #cv.namedWindow('thresholding', cv.WINDOW_NORMAL)

    #h_low = cv.getTrackbarPos('H_low','thresholding')
    #s_low = cv.getTrackbarPos('S_low','thresholding')
    #v_low = cv.getTrackbarPos('V_low','thresholding')
    #h_high = cv.getTrackbarPos('H_high','thresholding')
    #s_high = cv.getTrackbarPos('S_high','thresholding')
    #v_high = cv.getTrackbarPos('V_high','thresholding')

    lower_red = np.array([80,150,5],np.uint8)
    upper_red = np.array([255,255,255],np.uint8)

    #lower_red = np.array([h_low,s_low,v_low],np.uint8)
    #upper_red = np.array([h_high,s_high,v_high],np.uint8)

    binary_image = cv.inRange(hsv,lower_red,upper_red)

    return binary_image

def setCameraOffsets(clientID, radius, link_offset, angle, set_euler_angles):
    if set_euler_angles:
        inInts=[1]
    else:
        inInts=[0]

    inFloats=[radius,link_offset,angle,angle]
    emptyBuff = bytearray()

    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setCameraOffsets',[],inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)


def costFunction(image, reference_image, threshold):

    dist_weight = 0.2
    size_weight = 0.8

    binary_image = detectRed(image,threshold)
    ref_bin_image = detectRed(reference_image,threshold)
    count = cv.countNonZero(binary_image)
    ref_count = cv.countNonZero(ref_bin_image)


    #vector<vector<Point> > contours;
    #vector<Vec4i> hierarchy;

    im2,contours,hierarchy = cv.findContours( binary_image, 1, 2);

    if(len(contours) > 0):
        M = cv.moments(contours[0])

        if(M['m00'] > 0):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            ref_cx = 128
            ref_cy = 128

            distance = math.sqrt(math.pow(cx - ref_cx,2) + math.pow(cy - ref_cy,2))

            norm_distance = 1 - (distance/181)

            #print 'distance: ' + str(distance)
        else:
            #no contour detected, therefore return worst case = 0
            norm_distance = 0
    else:
        #no contour detected, therefore return worst case = 0
        norm_distance = 0

    cost = (count - ref_count)
    norm_cost = float(cost)/2000
    norm_count = float(count)/2000

    total_cost = size_weight*norm_cost + dist_weight*(norm_distance)

    count = count + norm_distance

    return total_cost,binary_image,norm_count,norm_distance


def deltaFunction(pixel_size, ref_size, blob_centre,ref_blob_centre, manip, ref_manip):

    dist_weight = 0.0
    size_weight = 0.8
    manip_weight = 0.0

    #ensure weights sum to 1
    dist_weight = dist_weight/(dist_weight+size_weight+manip_weight)
    size_weight = size_weight/(dist_weight+size_weight+manip_weight)
    manip_weight = manip_weight/(dist_weight+size_weight+manip_weight)

    # maxPixels = 256 * 256 * 2/3
    maxPixels = 200

    image_cx = 0.5
    image_cy = 0.5

    distance = 1 - math.sqrt(math.pow(blob_centre[0] - image_cx,2) + math.pow(blob_centre[1] - image_cy,2))
    ref_distance = 1 - math.sqrt(math.pow(ref_blob_centre[0] - image_cx,2) + math.pow(ref_blob_centre[1] - image_cy,2))

    distance_diff = distance - ref_distance

    norm_pixel_size = float(pixel_size)
    norm_ref_size = float(ref_size)

    pixel_diff = norm_pixel_size - norm_ref_size

    manip_diff = manip-ref_manip

    if not (pixel_size == 0.0):
        total_delta = size_weight*pixel_diff + dist_weight*distance_diff + manip_weight*manip_diff
    else:
        total_delta = 0.0

    return total_delta,distance_diff

def getCameraPositions(clientID):
    inInts=[]
    inFloats=[]
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'getCameraOffsets',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    nCameras = 9
    camera_positions = np.zeros([3,9])
    camera_poses = np.zeros([5,9])
    camera_orientations = []

    #print retFloats
    for i in range(0,nCameras):
        print(i)
        camera_orientations.append(np.zeros((4,)))

        camera_positions[0,i] = retFloats[(i*7)]
        camera_positions[1,i] = retFloats[(i*7)+1]
        camera_positions[2,i] = retFloats[(i*7)+2]

        camera_poses[0,i] = retFloats[(i*7)]
        camera_poses[1,i] = retFloats[(i*7)+1]
        camera_poses[2,i] = retFloats[(i*7)+2]
        camera_poses[3,i] = retFloats[(i*7)+3]
        camera_poses[4,i] = retFloats[(i*7)+4]
        # camera_poses[5,i] = retFloats[(i*7)+5]
        # camera_poses[6,i] = retFloats[(i*7)+6]


        # camera_orientations[0,i] = retFloats[(i*7)]
        # camera_orientations[1,i] = retFloats[(i*7)+1]
        # camera_orientations[2,i] = retFloats[(i*7)+2]
        # camera_orientations[3,i] = retFloats[(i*7)+3]

        camera_orientations[i][0] = retFloats[(i*7)+3]
        camera_orientations[i][1] = retFloats[(i*7)+4]
        camera_orientations[i][2] = retFloats[(i*7)+5]
        # camera_orientations[i][3] = retFloats[(i*7)+6]



    return camera_positions,np.array(camera_orientations), camera_poses

def showNImages(windowName, images):

    row1_stack  = np.hstack( (images[0],images[1],images[2]) )
    row2_stack  = np.hstack( (images[3],images[4],images[5]) )
    row3_stack  = np.hstack( (images[6],images[7],images[8]) )
    full_stack  = np.vstack( (row1_stack,row2_stack,row3_stack) )

    cv.imshow(windowName, full_stack)
    cv.waitKey(1)

def servo_target_camera(target_camera,step_size,clientID):

    inInts=target_camera
    inFloats=[step_size]
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movetoCamera',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    # print "retInts: ", retInts
    if(retInts[0] == 1):
        print ('servoing failed')
        return False
    else:
        return True

def getRawCostValues(clientID):
    inInts=[]
    inFloats=[]
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,
                                                                           'MoveFunctions',
                                                                           vrep.sim_scripttype_childscript,
                                                                           'getCostMatrix',
                                                                           [],[],[],emptyBuff,
                                                                           vrep.simx_opmode_oneshot_wait)

    delta_matrix = np.zeros([3,3])
    size_matrix = np.zeros([3,3])
    distance_matrix = np.zeros([3,3])
    manip_matrix = np.zeros([3,3])

    # print retFloats
    pixel_size = []
    blob_centre = []
    manip = []

    for i in range(0,9):
        pixel_size.append(retFloats[(i*4)])
        blob_centre.append([retFloats[(i*4)+1], retFloats[(i*4)+2]])
        manip.append(retFloats[(i*4)+3])

    #get the reference pixel size and manipulability (camera 5 indexed at 0)
    ref_size = pixel_size[4]
    ref_manip = manip[4]
    ref_blob_centre = blob_centre[4]

    return pixel_size, blob_centre, manip

def getCostValues(clientID):
    inInts=[]
    inFloats=[]
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,
                                                                           'MoveFunctions',
                                                                           vrep.sim_scripttype_childscript,
                                                                           'getCostMatrix',
                                                                           [],[],[],emptyBuff,
                                                                           vrep.simx_opmode_oneshot_wait)

    delta_matrix = np.zeros([3,3])
    size_matrix = np.zeros([3,3])
    distance_matrix = np.zeros([3,3])
    manip_matrix = np.zeros([3,3])

    # print retFloats
    pixel_size = []
    blob_centre = []
    manip = []

    for i in range(0,9):
        pixel_size.append(retFloats[(i*4)])
        blob_centre.append([retFloats[(i*4)+1], retFloats[(i*4)+2]])
        manip.append(retFloats[(i*4)+3])

    #get the reference pixel size and manipulability (camera 5 indexed at 0)
    ref_size = pixel_size[4]
    ref_manip = manip[4]
    ref_blob_centre = blob_centre[4]

    # print("ref blob size: ", ref_size)
    # print("ref blob_centre: ", ref_blob_centre)
    # print("ref manip: ", ref_manip)

    if(ref_size == 0.0):
        return delta_matrix, distance_matrix, manip_matrix

    for i in range(0,9):
        # print("i: ",i)
        # print("blob size: ", pixel_size[i])
        # print("blob_centre: ", blob_centre[i])
        # print("manip: ", manip[i])
        delta,norm_distance = deltaFunction(pixel_size[i], ref_size, blob_centre[i], ref_blob_centre, manip[i], ref_manip)
        delta_matrix[np.unravel_index(i, delta_matrix.shape)] = delta
        # size_matrix[np.unravel_index(i,size_matrix.shape)] = norm_size
        distance_matrix[np.unravel_index(i, distance_matrix.shape)] = norm_distance
        manip_matrix[np.unravel_index(i, manip_matrix.shape)] = manip[i]

    return delta_matrix, distance_matrix, manip_matrix, ref_size

def computeDirDerivative(direction_vectors,numerical_derivatives):

    print ("direction vector shape: ", direction_vectors.shape)
    print ("numerical_derivatives: ", numerical_derivatives.shape)

    direction_vectors_inv = np.linalg.pinv(direction_vectors.transpose())

    x = numerical_derivatives.reshape((1,len(numerical_derivatives)))

    derivative = x.dot(direction_vectors_inv)


    residuals = derivative.dot(direction_vectors.transpose()) - x


    print ("residuals: ", residuals)
    # print ("derivative: ", derivative.shape)

    return derivative

def weightedAverageVector(delta_matrix, camera_positions):

    flat_cost = delta_matrix.reshape([1,9])
    #print "flat_cost: ", flat_cost
    cost_by_pos = flat_cost*camera_positions
    #print "Cost*Pos:",cost_by_pos
    vdes = np.average(cost_by_pos,1)

    return vdes

def servo_xyz(xyz_delta, clientID):

    inInts=[]
    inFloats=xyz_delta
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'moveXYZdelta',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    if(retInts[0] == 1):
        print ('servoing failed')
        return False
    else:
        return True
    # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


    # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

#servo to pose where pose is [translation, quaternion]

def set_xyz(position, clientID):

    inInts=[]
    inFloats=position
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'moveXYZ',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    if(retInts[0] == 1):
        print ('servoing failed')
        return False
    else:
        return True

def setPose(pose, clientID):

    inInts=[]
    inFloats= pose
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setPose',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    # print "retInts: ", retInts
    if(retInts[0] == 0):
        return True
    else:
        print ('servoing failed')
        return False

def servoPose(pose, clientID):

    inInts=[]
    inFloats= pose
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movePose',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    # print "retInts: ", retInts
    if(retInts[0] == 0):
        return True
    else:
        print ('servoing failed')
        return False
    # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


    # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

def servoPoseEuler(pose, clientID):

    inInts=[]
    inFloats= pose
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'movePoseEuler',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    # print "retInts: ", retInts
    if(retInts[0] == 0):
        return True
    else:
        print ('servoing failed')
        return False
    # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


    # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

def setPoseEuler(pose, clientID):

    inInts=[]
    inFloats= pose
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'setPoseEuler',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    # print "retInts: ", retInts
    if(retInts[0] == 0):
        return True
    else:
        print ('servoing failed')
        return False
    # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


    # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


def nothing(x):
    pass

#Compute the weighted average of quaternions
def weighted_avg_quaternions(q_array,w):

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

def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / N

def execute(clientID):
    print ('Program started')
    #vrep.simxFinish(-1) # just in case, close all opened connections
    #clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    nCameras = 9
    #res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)
    plt.clf()
    fig = plt.figure(1)
    if clientID!=-1:
        print ('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
        # print('Started opencv gui')

        # Now retrieve streaming data (i.e. in a non-blocking fashion):
        startTime=time.time()

        ref_index = 4 #index of middle camera

        plot_array1 = []
        max_size = []
        max_distance = []
        plot_array2 = []

        plt.ion()

        count = 0
        counts = []

        camera_positions, camera_orientations, camera_poses = getCameraPositions(clientID)

        queue_size = 10
        delta_magnitude_queue = deque(np.zeros(queue_size))
        current_delta_mag = 1e5
        avg_delta_magnitude = 0.0
        tolerance = 0.000001

        while abs(avg_delta_magnitude - current_delta_mag) > tolerance:

            delta_matrix, distance_matrix, manipulability_matrix, ref_size = getCostValues(clientID)
            delta_flat = np.delete(delta_matrix.reshape((1,9)),4,None)


            camera_vectors = (np.delete(camera_poses,4,1)).transpose()
            camera_vector_mags = np.linalg.norm(camera_vectors,None,1)

            camera_unit_vectors = camera_vectors/camera_vector_mags[:,np.newaxis]

            numerical_derivative = np.divide(delta_flat,camera_vector_mags)

            print ('camera_unit_vectors:', camera_unit_vectors)
            print ('numerical_derivative:', numerical_derivative)

            # derivative = computeDirDerivative(camera_unit_vectors,numerical_derivative)
            derivative = computeDirDerivative(camera_unit_vectors,numerical_derivative)
            print ('derivative:', derivative)
            print ('derivative:', derivative.shape)

            # delta_matrix[1,1] = 0.00001
            # print ('gradient for current step')
            # print (delta_matrix)

            # print ('camera poses: ')
            # print camera_poses
            # print camera_poses.shape
            #
            # print ('camera positions: ')
            # print camera_orientations

            # camera_poses =

            if (np.sum(delta_matrix) == 0.0):
                break
            else:

                # translation = weightedAverageVector(delta_matrix,camera_positions)
                # quaternion = weighted_avg_quaternions(camera_orientations,delta_matrix.reshape((1,9))[0])
                # print "weighted translation: ",derivative
                # print "weighted quaternion: ",quaternion

                # step_size = np.array([100.0,25.0,25.0])
                # step_size = np.array([0.1,0.1,0.1,10.0,10.0,10.0])
                step_size = 0.05
                #step_size = 0.1
                # q_step_size = 0.005
                # Qref = pyQuat(camera_orientations[4])
                # print quaternion
                # Qfinal = pyQuat(quaternion)
                # Qslerp = pyQuat.slerp(Qref,Qfinal,q_step_size)
                # print "Qref", Qref
                # print "Qfinal", Qfinal
                # print "Qslerp", Qslerp

                # scaled_translation = step_size*translation
                # scaled_translation = step_size*derivative
                accum_step_size = 0
                step_size = 0.01
                for i in range(30):

                    accum_step_size = accum_step_size + step_size
                    print "accum_step_size: ", accum_step_size

                    pose = step_size*derivative
                    pose = np.append(pose,0) #add 0 angle for yaw
                    pose = pose.reshape((6,))

                    servoPoseEuler(pose,clientID)

                    _,_,_,ref_size = getCostValues(clientID)

                    plot_array1.append(ref_size)
                    plot_array2.append(accum_step_size)


                    #reset position back to start
                    # setPoseEuler(camera_poses,clientID)

                # plt.subplot(1,2,1)
                fig.clf()
                plt.plot(plot_array2,plot_array1)
                # plt.subplot(1,2,2)
                # plt.plot(plot_array2)
                plt.pause(0.0001)
                # print "scaled translation: ", pose

                # pose = np.concatenate((scaled_translation,Qslerp.elements))
                # pose = np.concatenate((scaled_translation.reshape((3,)),Qref.elements))
                # pose = np.concatenate((scaled_translation.reshape((3,)),Qref.elements))

                #servo_xyz(pose, clientID)
                # print "Pose: ", pose
                # print pose.shape
                # # servoPoseEuler(pose,clientID)
                # setPoseEuler(pose,clientID)

                # delta_flat = delta_matrix.reshape((9))
                # current_delta_mag = np.sqrt(delta_flat.dot(delta_flat))
                #
                # delta_magnitude_queue.appendleft(current_delta_mag)
                # delta_magnitude_queue.pop()
                # avg_delta_magnitude = sum(delta_magnitude_queue)/queue_size

                # print "Current delta magnitude: ", current_delta_mag
                # print "Average delta magnitude: ", avg_delta_magnitude

                # plot_array1.append(delta_matrix[max_index_row_col])
                # plot_array1.append(ref_size)
                # max_size.append(size_matrix[max_index_row_col])
                # max_distance.append(distance_matrix[max_index_row_col])
                # plot_array2.append(manipulability_matrix[max_index_row_col])
                # plot_array2.append(avg_delta_magnitude)
                counts.append(count)
                count = count + 1

                # time.sleep(1)

                #axis_direction = np.matrix((1,-1,1))
                #xyz_vector = step_size*np.hstack((np.zeros((1,1)),max_unit_vector))
                #xyz_vector = np.multiply(xyz_vector,axis_direction)

                #servo_target_camera(max_indexes.tolist(),step_size,clientID)

                # plt.subplot(1,2,1)
                # plt.plot(plot_array2,plot_array1)
                # # plt.subplot(1,2,2)
                # # plt.plot(plot_array2)
                # plt.pause(0.0001)

    else:
        print ('Failed connecting to remote API server')

    print ('cost within tolerance, finished')

#if __name__=="__main__":
    #move_to_see()
    #cProfile.run('move_to_see()')
