# -*- coding: utf-8 -*-
"""
Created on Thu Apr 25 06:59:02 2019

@author: Sneha
"""
import numpy as np
import vrep
import sys

text_file = open("FinalPath.txt", "r")
lines = (text_file.read()).split('  ')
text_file.close()
final_path=[]
for i in lines:
    if(i!=''):
        k=i.split(',')
        for j in range(5):
            k[j]=float(k[j])
    
        final_path.append(k)

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
goal=[final_path[len(final_path)-1][0],final_path[len(final_path)-1][1]]
if clientID!=-1:
    print ('Connected to remote API server')
    [_,left_wheel] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot_wait);

    [ _,right_wheel]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot_wait);
   
    [_,reference_frame]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_oneshot_wait);

    
    [_,turtlebot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_oneshot_wait);

    [_,start_position]=vrep.simxGetObjectPosition(clientID, turtlebot,-1,vrep.simx_opmode_oneshot_wait);

    
    start = np.double([start_position[0],start_position[1]])
    
    s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait);

    
    s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait);

    err,simtime = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_streaming)

    for i in range(0,len(final_path)-1):

        [_,position]=vrep.simxGetObjectPosition(clientID, turtlebot,-1,vrep.simx_opmode_oneshot_wait);
        [_,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);

           
        position = np.double(position);
        theta_req = final_path[i][2]

        if(np.abs(theta_req - final_path[i+1][2])>0.1 ):
            while  np.abs(theta[2] - theta_req)> 0.05:

                if theta[2] < theta_req :

                    if(np.abs(theta[2] - theta_req)<1):
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,-1,vrep.simx_opmode_oneshot_wait);
                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,1,vrep.simx_opmode_oneshot_wait);

                    else:
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,-2,vrep.simx_opmode_oneshot_wait);

                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,2,vrep.simx_opmode_oneshot_wait);
                
                else:
 
                    if(np.abs(theta[2] - theta_req)<1):
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,1,vrep.simx_opmode_oneshot_wait);
                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-1,vrep.simx_opmode_oneshot_wait);

                    else:
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,2,vrep.simx_opmode_oneshot_wait);
                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-2,vrep.simx_opmode_oneshot_wait);
               
                [_,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
       
        [_,position]=vrep.simxGetObjectPosition(clientID, turtlebot,-1,vrep.simx_opmode_oneshot_wait);
        position = np.double(position);

        [_,position]=vrep.simxGetObjectPosition(clientID, turtlebot,-1,vrep.simx_opmode_oneshot_wait);
        position = np.double(position);
#        print(np.abs(position[0] - goal[0]),' ',np.abs(position[1] - goal[1]))
        if np.abs(position[0] - goal[0]) < 0.2 and  np.abs(position[1] - goal[1]) < 0.2:
                break;
                
   
        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,final_path[i][3],vrep.simx_opmode_oneshot_wait);
        
        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,final_path[i][4],vrep.simx_opmode_oneshot_wait);
        
        _,simtime1 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
        _,simtime2 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
        while (simtime2 - simtime1) < 1.14:
            _,simtime2 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
            


    print("Location Reached");
    s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait);
    
    s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait);
    
    vrep.simxFinish(-1);

    vrep.simxFinish(clientID)
else:
    print('Connection unsuccessful!')
    sys.exit("Could not connect")