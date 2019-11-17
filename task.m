clc;
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


if (clientID>-1)
    disp('Connected to remote API server');
    %Handle
    [returnCode,steer]=vrep.simxGetObjectHandle(clientID,'steer_joint',vrep.simx_opmode_blocking);
    [returnCode,motor]=vrep.simxGetObjectHandle(clientID,'motor_joint',vrep.simx_opmode_blocking);
    
    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
    [returnCode,front_sensor]=vrep.simxGetObjectHandle(clientID,'UltrasonicSensor_f',vrep.simx_opmode_blocking);
    [returnCode,left_sensor]=vrep.simxGetObjectHandle(clientID,'UltrasonicSensor_l',vrep.simx_opmode_blocking);
    [returnCode,right_sensor]=vrep.simxGetObjectHandle(clientID,'UltrasonicSensor_r',vrep.simx_opmode_blocking);
    
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
%     [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_motor,0.1,vrep.simx_opmode_blocking);

    
    [returnCode,detectionState_f,detectedPoint_f,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_l,detectedPoint_l,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_r,detectedPoint_r,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_streaming);
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

    for i=1:400
        [returnCode,detectionState_f,detectedPoint_f,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer);
        [returnCode,detectionState_l,detectedPoint_l,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_buffer);
        [returnCode,detectionState_r,detectedPoint_r,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_buffer);
        dis_f=norm(detectedPoint_f);
        dis_l=norm(detectedPoint_l);
        dis_r=norm(detectedPoint_r);
        %disp(dis_f);
        %disp(dis_l);
        disp(dis_r);
        
        if dis_f>0.02&&dis_f<1.5
            steer_angle=0.5;
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
            disp(steer_angle);
        end
                
        if dis_l>0.02&&dis_l<0.5
            steer_angle=-0.2;
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
        elseif steer_angle == -0.2 && dis_l > 0.6
            steer_angle=0;
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
            %disp(steer_angle);
            %disp(dis_l);
        end
        if dis_r>0.02&&dis_r<0.5
            steer_angle=0.2;
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
        elseif steer_angle == 0.2 && dis_r > 0.6
            steer_angle=0;
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,steer,steer_angle,vrep.simx_opmode_blocking);
            %disp(steer_angle);
            %disp(dis_r);
        end
        %disp([norm(detectedPoint_f),norm(detectedPoint_l),norm(detectedPoint_r)]);
        %[returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
        %imshow(image);
        pause(0.1)
    end
    
    %brake part
    brake_force=100;
    [returnCode]=vrep.simxSetJointForce(clientID,fl_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,fr_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,bl_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,br_brake,brake_force,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,motor,0,vrep.simx_opmode_blocking);

    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor,0,vrep.simx_opmode_blocking);

    vrep.simxFinish(clientID);
end

vrep.delete(); % call the destructor!
disp('Program ended');