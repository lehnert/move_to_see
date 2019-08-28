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
    print ('--------------------------------------------------------------')
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
import math
import cProfile
from PIL import Image as I
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


def costFunctionv2(pixel_size, ref_size, cx, cy):

    dist_weight = 0.2
    size_weight = 0.8

    ref_cx = 128
    ref_cy = 128
            
    distance = math.sqrt(math.pow(cx - ref_cx,2) + math.pow(cy - ref_cy,2))

    norm_distance = 1 - (distance/181)

    cost = (pixel_size - ref_size)
    norm_cost = float(cost)/2000
    norm_count = float(pixel_size)/2000
    
    total_cost = size_weight*norm_cost + dist_weight*(norm_distance)

    count = count + norm_distance

    return total_cost,norm_count,norm_distance

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
    if(retInts[0] == 1):
        print ('servoing failed')
        return False
    else:
        return True

def getCostMatrix(clientID):
    inInts=target_camera
    inFloats=[step_size]
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,
                                                                           'MoveFunctions',
                                                                           vrep.sim_scripttype_childscript,
                                                                           'getObjectDetails',
                                                                           inInts,inFloats,[],emptyBuff,
                                                                           vrep.simx_opmode_oneshot_wait)

    cost_matrix = np.zeros([3,3])

    ref_size = retFloats(4*3)
    
    for i in range(0,9):
        pixel_size = retFloats[(i*3)]
        cx = retFloats[(i*3)+1]
        cy = retFloats[(i*3)+2]

                    
        rows = cost_matrix.shape[0]
        cols = cost_matrix.shape[1]

        cost_matrix[np.unravel_index(i,cost_matrix.shape)] = costFunctionv2(pixel_size,ref_size,cx,cy)
        
        for i in range(0,rows):
            for j in range(0,cols):

                    norm_cost = 0.0
                    cost,binary_image,pixel_count,norm_distance = costFunction(images[(i*rows) + j],images[ref_index],red_threshold)
                    #if(cost > 0):
                    #    norm_cost = float(cost)/float(resolution[0]*resolution[1])
                    #else:
                    #    norm_cost = 0.0
                    cost_matrix[i,j] = cost
    
    if(retInts[0] == 1):
        print ('servoing failed')
        return False
    else:
        return True
    

def servo_xyz(xyz_delta, robot_handle, clientID):

    inInts=[robotHandle]
    inFloats=xyz_delta
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'moveXYZ',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    if(retInts[0] == 1):
        print ('servoing failed')
        return False
    else:
        return True
    # res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getRobotState',[robot_handle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


    # res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

def nothing(x):
    pass


def move_to_see():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    nCameras = 9
    res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)

    if clientID!=-1:
        print ('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
        print('Started opencv gui')
        cv.startWindowThread()
        cv.namedWindow('image_array', cv.WINDOW_NORMAL)
        cv.namedWindow('threshold_array', cv.WINDOW_NORMAL)

        #cv.namedWindow('thresholding', cv.WINDOW_NORMAL)
        #cv.createTrackbar('H_low','thresholding',0,255,nothing)
        #cv.createTrackbar('S_low','thresholding',0,255,nothing)
        #cv.createTrackbar('V_low','thresholding',0,255,nothing)
        #cv.createTrackbar('H_high','thresholding',0,255,nothing)
        #cv.createTrackbar('S_high','thresholding',0,255,nothing)
        #cv.createTrackbar('V_high','thresholding',0,255,nothing)

        vision_sensor_handles = []
        for i in range(0,nCameras):
            res,vision_sensor_handle=vrep.simxGetObjectHandle(clientID,'Vision_sensor'+str(i),vrep.simx_opmode_blocking)
            vision_sensor_handles.append(vision_sensor_handle)
            if res==vrep.simx_return_ok:
                print ('Got handle for camera #: '+str(i))
                returnCode,resolution,image=vrep.simxGetVisionSensorImage(clientID,vision_sensor_handles[i],0,vrep.simx_opmode_streaming)
                if returnCode == vrep.simx_return_ok:
                    print ('got first image for camera #: ' +str(i))
                        # # print (image)
            else:
                print ('Remote API function call returned with error code: ',res)

        time.sleep(2)


        # Now retrieve streaming data (i.e. in a non-blocking fashion):
        startTime=time.time()
        # vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming) # Initialize streaming # Initialize streaming

        red_threshold = 55 #detection threshold
        ref_index = 4 #index of middle camera

        max_costs = []
        max_pixel_count = []
        max_distance = []
     

        plt.ion()
        
        #fig = plt.figure()
        #ax = fig.add_subplot(111)

        #line1, = ax.plot([],[],'r-')
        
        count = 0
        counts = []

     
        while time.time()-startTime < 480:
        # for t in [1]:

            #time.sleep(5)

            images = []
            thresholdImages = []
            for i in range(0,nCameras):
                returnCode,resolution,image=vrep.simxGetVisionSensorImage(clientID,vision_sensor_handles[i],0,vrep.simx_opmode_buffer)
                if returnCode == vrep.simx_return_ok:
                    # print (image)
                    image_byte_array = array.array('b', image)
                    image_buffer = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
                    img2 = np.asarray(image_buffer)
                    imgRGB = cv.cvtColor(img2,cv.COLOR_BGR2RGB)
                    # img2  = img2[:,:,::-1]
                    imgRGBFlipped = cv.flip(imgRGB,1)
                    # img2 = img2.transpose((2,1,0))
                    images.append(imgRGBFlipped)

            cost_matrix = np.zeros([3,3])
            pixel_count_matrix = np.zeros([3,3])
            distance_matrix = np.zeros([3,3])
            
            rows = cost_matrix.shape[0]
            cols = cost_matrix.shape[1]
            for i in range(0,rows):
                for j in range(0,cols):

                    norm_cost = 0.0
                    cost,binary_image,pixel_count,norm_distance = costFunction(images[(i*rows) + j],images[ref_index],red_threshold)
                    #if(cost > 0):
                    #    norm_cost = float(cost)/float(resolution[0]*resolution[1])
                    #else:
                    #    norm_cost = 0.0
                    cost_matrix[i,j] = cost
                    pixel_count_matrix[i,j] = pixel_count
                    distance_matrix[i,j] = norm_distance


                    thresholdImages.append(binary_image)

                    # cv.imshow('image'+str(i),img2)
                    # cv.waitKey(1)
                # imageAcquisitionTime=vrep.simxGetLastCmdTime(clientID)
            cost_matrix[1,1] = 0.0001
        
            print ('Cost Function for current step')
            print (cost_matrix)

            origin_offset = np.matrix((1,1))
            max_index = np.argmax(cost_matrix)
            max_indexes = np.flip(np.argsort(cost_matrix, None),0) + 1
            max_unit_index = np.unravel_index(max_index,cost_matrix.shape)
            max_unit_vector = np.unravel_index(np.argmax(cost_matrix),cost_matrix.shape) - origin_offset
            print ('xy unit vector: ')
            print (max_indexes)

            max_costs.append(cost_matrix[max_unit_index])
            max_pixel_count.append(pixel_count_matrix[max_unit_index])
            max_distance.append(distance_matrix[max_unit_index])
            counts.append(count)
            count = count + 1

            step_size = 0.1
            axis_direction = np.matrix((1,-1,1))
            xyz_vector = step_size*np.hstack((np.zeros((1,1)),max_unit_vector))
            xyz_vector = np.multiply(xyz_vector,axis_direction)

            servo_target_camera(max_indexes.tolist(),step_size,clientID)
            # servo_xyz(xyz_vector.tolist()[0],robotHandle,clientID)
            showNImages('image_array', images)
            showNImages('threshold_array', thresholdImages)

            plt.subplot(1,2,1)
            plt.plot(max_pixel_count)
            plt.subplot(1,2,2)
            plt.plot(max_distance)
            plt.pause(0.05)

            #line1.set_xdata(counts)
            #line1.set_ydata(max_costs)
            #fig.canvas.draw()
            

        # Now send some data to V-REP in a non-blocking fashion:
        vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)

        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')

    # cv.destroyAllWindows()
    print ('Program ended')

if __name__=="__main__":
    
    cProfile.run('move_to_see()')
