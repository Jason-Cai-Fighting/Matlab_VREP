clc;
a=readfis('car_control');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


if (clientID>-1)
    disp('Connected to remote API server');
    %Handle
    [returnCode,steer]=vrep.simxGetObjectHandle(clientID,'steer_joint',vrep.simx_opmode_blocking);
    [returnCode,motor]=vrep.simxGetObjectHandle(clientID,'motor_joint',vrep.simx_opmode_blocking);
    [returnCode,car_pos]=vrep.simxGetObjectHandle(clientID,'CarPos',vrep.simx_opmode_blocking);
    [returnCode,tar_pos]=vrep.simxGetObjectHandle(clientID,'TarPos',vrep.simx_opmode_blocking);
    
    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
    [returnCode,front_sensor]=vrep.simxGetObjectHandle(clientID,'Sensor_f',vrep.simx_opmode_blocking);
    [returnCode,left_sensor]=vrep.simxGetObjectHandle(clientID,'Sensor_l',vrep.simx_opmode_blocking);
    [returnCode,right_sensor]=vrep.simxGetObjectHandle(clientID,'Sensor_r',vrep.simx_opmode_blocking);
    [returnCode,leftf_sensor]=vrep.simxGetObjectHandle(clientID,'Sensor_lf',vrep.simx_opmode_blocking);
    [returnCode,rightf_sensor]=vrep.simxGetObjectHandle(clientID,'Sensor_rf',vrep.simx_opmode_blocking);
    
    [returnCode,fl_brake]=vrep.simxGetObjectHandle(clientID,'fl_brake_joint',vrep.simx_opmode_blocking);
    [returnCode,fr_brake]=vrep.simxGetObjectHandle(clientID,'fr_brake_joint',vrep.simx_opmode_blocking);
    [returnCode,bl_brake]=vrep.simxGetObjectHandle(clientID,'bl_brake_joint',vrep.simx_opmode_blocking);
    [returnCode,br_brake]=vrep.simxGetObjectHandle(clientID,'br_brake_joint',vrep.simx_opmode_blocking);
    
    max_steer_angle=0.5235987;
    motor_torque=60;
    dVel=1;
    dSteer=0.1;
    steer_angle=0;
    motor_velocity=dVel*3;
    brake_force=0;
    %code
    %brake part
    [returnCode]=vrep.simxSetJointForce(clientID,fl_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,fr_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,bl_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,br_brake,brake_force,vrep.simx_opmode_blocking);
        
    [returnCode]=vrep.simxSetJointForce(clientID,motor,motor_torque,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor,motor_velocity,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
    
    [returnCode,rel_pos]=vrep.simxGetObjectPosition(clientID,tar_pos,car_pos,vrep.simx_opmode_streaming);
%     [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_motor,0.1,vrep.simx_opmode_blocking);

    
    [returnCode,detectionState_f,detectedPoint_f,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_l,detectedPoint_l,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_r,detectedPoint_r,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_lf,detectedPoint_lf,~,~]=vrep.simxReadProximitySensor(clientID,leftf_sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_rf,detectedPoint_rf,~,~]=vrep.simxReadProximitySensor(clientID,rightf_sensor,vrep.simx_opmode_streaming);
%     [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
    
%     tic
%     for i=1:50  
%         [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer);
%         [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
%         imshow(image);
%         disp(toc);
%         %disp(norm(detectedPoint));
%         pause(0.1);
%     end
    
%     [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_motor,0,vrep.simx_opmode_blocking);

    for i=1:2000
        [returnCode,detectionState_f,detectedPoint_f,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer);
        [returnCode,detectionState_l,detectedPoint_l,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_buffer);
        [returnCode,detectionState_r,detectedPoint_r,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_buffer);
        [returnCode,detectionState_lf,detectedPoint_lf,~,~]=vrep.simxReadProximitySensor(clientID,leftf_sensor,vrep.simx_opmode_buffer);
        [returnCode,detectionState_rf,detectedPoint_rf,~,~]=vrep.simxReadProximitySensor(clientID,rightf_sensor,vrep.simx_opmode_buffer);
        [returnCode,rel_pos]=vrep.simxGetObjectPosition(clientID,tar_pos,car_pos,vrep.simx_opmode_buffer);
        %processing distance
        dis_l=norm(detectedPoint_l);
        if detectionState_l == 0
            dis_l = 2;
        end
        
        dis_lf=norm(detectedPoint_lf);
        if detectionState_lf == 0
            dis_lf = 2;
        end
        
        dis_f=norm(detectedPoint_f);        
        if detectionState_f == 0
            dis_f = 2;
        end
        
        dis_r=norm(detectedPoint_r);
        if detectionState_r == 0
            dis_r = 2;
        end
        
        dis_rf=norm(detectedPoint_rf);
        if detectionState_rf == 0
            dis_rf = 2;
        end
        disp(dis_l);
        disp(dis_lf);
        disp(dis_f);
        disp(dis_rf);
        disp(dis_r);    
        %deal with the target position
        dis = norm(rel_pos);
        if dis ~=0 && dis < 0.1
            disp('Reach target position!');
            break;
        end
        tar_p = atan2d(rel_pos(2),rel_pos(1))-90;
        if tar_p < -150 && tar_p >= -180
            tar_p = -150;
        elseif tar_p < -180 && tar_p >= -210
            tar_p = 150;
        elseif tar_p < -210
            tar_p = tar_p + 360;
        end
%         disp(tar_p);
        [returnvalue]=evalfis([dis_l dis_f dis_r tar_p dis_lf dis_rf], a);
        steer_angle = returnvalue(1)
        motor_velocity = returnvalue(2);
        times = 5;
        if dis < 1.5
            times = 2;
        end
        motor_velocity = motor_velocity * times;
        steer_angle = -(steer_angle - 90 )/180*pi;
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor,motor_velocity,vrep.simx_opmode_blocking);
        
        pause(0.1)
    end
    
    %brake part
    brake_force=100;
    [returnCode]=vrep.simxSetJointForce(clientID,fl_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,fr_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,bl_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,br_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,motor,-1,vrep.simx_opmode_blocking);

    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor,0,vrep.simx_opmode_blocking);

    vrep.simxFinish(clientID);
end

vrep.delete(); % call the destructor!
disp('Program ended');