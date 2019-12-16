import vrep
import time
from math import atan2,pi
from numpy import linalg as LA
from CarFuzzyControl import car
import numpy as np
import matplotlib.pyplot as mlp

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')

    # Handle
    returnCode,steer=vrep.simxGetObjectHandle(clientID,'steer_joint',vrep.simx_opmode_blocking)
    returnCode,motor=vrep.simxGetObjectHandle(clientID,'motor_joint',vrep.simx_opmode_blocking)
    returnCode,car_pos=vrep.simxGetObjectHandle(clientID,'CarPos',vrep.simx_opmode_blocking)
    returnCode,tar_pos=vrep.simxGetObjectHandle(clientID,'TarPos',vrep.simx_opmode_blocking)
    
    returnCode,camera=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking)
    returnCode,front_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_f',vrep.simx_opmode_blocking)
    returnCode,left_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_l',vrep.simx_opmode_blocking)
    returnCode,right_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_r',vrep.simx_opmode_blocking)
    returnCode,leftf_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_lf',vrep.simx_opmode_blocking)
    returnCode,rightf_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_rf',vrep.simx_opmode_blocking)
    
    returnCode,fl_brake=vrep.simxGetObjectHandle(clientID,'fl_brake_joint',vrep.simx_opmode_blocking)
    returnCode,fr_brake=vrep.simxGetObjectHandle(clientID,'fr_brake_joint',vrep.simx_opmode_blocking)
    returnCode,bl_brake=vrep.simxGetObjectHandle(clientID,'bl_brake_joint',vrep.simx_opmode_blocking)
    returnCode,br_brake=vrep.simxGetObjectHandle(clientID,'br_brake_joint',vrep.simx_opmode_blocking)

    returnCode,camera=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking)
    
    #basic parameter
    max_steer_angle=0.5235987;
    motor_torque=60;
    dVel=1;
    dSteer=0.1;
    steer_angle=0;
    motor_velocity=dVel*3;
    brake_force=0;
    
    #set the brake part
    returnCode=vrep.simxSetJointForce(clientID,fl_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,fr_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,bl_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,br_brake,brake_force,vrep.simx_opmode_blocking)
    
    #drive car
    returnCode=vrep.simxSetJointForce(clientID,motor,motor_torque,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointTargetVelocity(clientID,motor,motor_velocity,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking)

    #read data from vrep
    returnCode,detectionState_l,detectedPoint_l,OH,SNV=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_streaming)    
    returnCode,detectionState_lf,detectedPoint_lf,OH,SNV=vrep.simxReadProximitySensor(clientID,leftf_sensor,vrep.simx_opmode_streaming)
    returnCode,detectionState_f,detectedPoint_f,OH,SNV=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_streaming)
    returnCode,detectionState_rf,detectedPoint_rf,OH,SNV=vrep.simxReadProximitySensor(clientID,rightf_sensor,vrep.simx_opmode_streaming)
    returnCode,detectionState_r,detectedPoint_r,OH,SNV=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_streaming)
    returnCode,rel_pos=vrep.simxGetObjectPosition(clientID,tar_pos,car_pos,vrep.simx_opmode_streaming);
    returnCode,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera,0,vrep.simx_opmode_streaming)

    for i in range(1,1000):
        #read sensor data
        returnCode,detectionState_f,detectedPoint_f,OH,SNV=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer)
        returnCode,detectionState_l,detectedPoint_l,OH,SNV=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_buffer)
        returnCode,detectionState_r,detectedPoint_r,OH,SNV=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_buffer)
        returnCode,detectionState_lf,detectedPoint_lf,OH,SNV=vrep.simxReadProximitySensor(clientID,leftf_sensor,vrep.simx_opmode_buffer)
        returnCode,detectionState_rf,detectedPoint_rf,OH,SNV=vrep.simxReadProximitySensor(clientID,rightf_sensor,vrep.simx_opmode_buffer)
        returnCode,rel_pos=vrep.simxGetObjectPosition(clientID,tar_pos,car_pos,vrep.simx_opmode_buffer)
        returnCode,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera,0,vrep.simx_opmode_buffer)
        
        #sensor data processing
        dis_l=LA.norm(detectedPoint_l)
        if detectionState_l == 0:
            dis_l = 2
        
        dis_lf=LA.norm(detectedPoint_lf);
        if detectionState_lf == 0:
            dis_lf = 2
        
        dis_f=LA.norm(detectedPoint_f);        
        if detectionState_f == 0:
            dis_f = 2
        
        dis_r=LA.norm(detectedPoint_r);
        if detectionState_r == 0:
            dis_r = 2
        
        dis_rf=LA.norm(detectedPoint_rf);
        if detectionState_rf == 0:
            dis_rf = 2
        #destination approach detection
        dis = LA.norm(rel_pos)
        if dis !=0 and dis < 0.1:
            print('Reach target position!')
            break
        #target position calculation
        tar_p = atan2(rel_pos[1],rel_pos[0])/pi*180-90
        if tar_p < -150 and tar_p >= -180:
            tar_p = -150
        elif tar_p < -180 and tar_p >= -210 :
            tar_p = 150
        elif tar_p < -210:
            tar_p = tar_p + 360
        #display sensor data
        input = [dis_l,dis_lf,dis_f,dis_rf,dis_r,tar_p]
        print(input)
        #input sensor data to fuzzy controller
        car.inputs({'dis_l':dis_l ,'dis_lf':dis_lf ,'dis_f':dis_f ,'dis_rf':dis_rf ,'dis_r':dis_r ,'goal_position':tar_p})
        car.compute()
        steer_angle = car.output['steer_angle']
        motor_velocity = car.output['speed']
        print(steer_angle,motor_velocity)
        #when approaching destination, slow down
        times = 5
        if dis < 1.5:
            times = 2
        #calculate velocity and angle used in vrep
        motor_velocity = motor_velocity * times
        steer_angle = -(steer_angle - 90 )/180*pi
        #set value in vrep
        returnCode=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking)
        returnCode=vrep.simxSetJointTargetVelocity(clientID,motor,motor_velocity,vrep.simx_opmode_blocking)
        
        #image processing
        im = np.array(image, dtype=np.uint8)
        im.resize([256,256,3])
        mlp.imshow(im)
        
        time.sleep(0.1)
    #stop the car
    brake_force=100
    returnCode=vrep.simxSetJointForce(clientID,fl_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,fr_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,bl_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,br_brake,brake_force,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointForce(clientID,motor,-1,vrep.simx_opmode_blocking)
    returnCode=vrep.simxSetJointTargetVelocity(clientID,motor,0,vrep.simx_opmode_blocking)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')