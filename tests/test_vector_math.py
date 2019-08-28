import numpy as np
import vrep

def getCameraPositions(clientID):
    inInts=[]
    inFloats=[]
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'MoveFunctions',vrep.sim_scripttype_childscript,'getCameraOffsets',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    nCameras = 9
    camera_positions = np.zeros([3,9])

    print retFloats
    for i in range(0,nCameras-1):
        camera_positions[0,i] = retFloats[(i*3)]
        camera_positions[1,i] = retFloats[(i*3)+1]
        camera_positions[2,i] = retFloats[(i*3)+2]

    return camera_positions

if __name__=="__main__":

    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    nCameras = 9
    res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5',vrep.simx_opmode_oneshot_wait)

    if clientID!=-1:
        print ('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)


    print getCameraPositions(clientID)
    # n_cameras = 9
    # cost_matrix = np.zeros([1,n_cameras])
    #
    # cost_matrix[0,0] = 0
    # cost_matrix[0,1] = -5
    # cost_matrix[0,2] = -10
    #
    # cost_matrix[0,3] = 5
    # cost_matrix[0,4] = 0
    # cost_matrix[0,5] = -5
    #
    # cost_matrix[0,6] = 10
    # cost_matrix[0,7] = 4
    # cost_matrix[0,8] = -3
    #
    # camera_origins = np.zeros([3,n_cameras])
    #
    # camera_origins[0,0:8] = 1
    # camera_origins[1,0:8] = 2
    # camera_origins[2,0:8] = 1
    #
    # # camera_origins[1,0] = 2
    # # camera_origins[2,0] = 1
    #
    # # print (camera_origins)
    # # print (cost_matrix)
    # print (cost_matrix*camera_origins)
    # print
    # vdes = np.average(cost_matrix*camera_origins,1)
    # print (vdes)
