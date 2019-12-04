import vrep
import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')

    # Handle
    returnCode,steer=vrep.simxGetObjectHandle(clientID,'steer_joint',vrep.simx_opmode_blocking);
    returnCode,motor=vrep.simxGetObjectHandle(clientID,'motor_joint',vrep.simx_opmode_blocking);
    returnCode,car_pos=vrep.simxGetObjectHandle(clientID,'CarPos',vrep.simx_opmode_blocking);
    returnCode,tar_pos=vrep.simxGetObjectHandle(clientID,'TarPos',vrep.simx_opmode_blocking);
    
    returnCode,camera=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
    returnCode,front_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_f',vrep.simx_opmode_blocking);
    returnCode,left_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_l',vrep.simx_opmode_blocking);
    returnCode,right_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_r',vrep.simx_opmode_blocking);
    returnCode,leftf_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_lf',vrep.simx_opmode_blocking);
    returnCode,rightf_sensor=vrep.simxGetObjectHandle(clientID,'Sensor_rf',vrep.simx_opmode_blocking);
    
    returnCode,fl_brake=vrep.simxGetObjectHandle(clientID,'fl_brake_joint',vrep.simx_opmode_blocking);
    returnCode,fr_brake=vrep.simxGetObjectHandle(clientID,'fr_brake_joint',vrep.simx_opmode_blocking);
    returnCode,bl_brake=vrep.simxGetObjectHandle(clientID,'bl_brake_joint',vrep.simx_opmode_blocking);
    returnCode,br_brake=vrep.simxGetObjectHandle(clientID,'br_brake_joint',vrep.simx_opmode_blocking);

    #basic parameter
    max_steer_angle=0.5235987;
    motor_torque=60;
    dVel=1;
    dSteer=0.1;
    steer_angle=0;
    motor_velocity=dVel*3;
    brake_force=0;
    
    #drive car
    returnCode=vrep.simxSetJointForce(clientID,motor,motor_torque,vrep.simx_opmode_blocking);
    returnCode=vrep.simxSetJointTargetVelocity(clientID,motor,motor_velocity,vrep.simx_opmode_blocking);
    returnCode=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);


    for i in range(1,100):
        time.sleep(0.01)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')