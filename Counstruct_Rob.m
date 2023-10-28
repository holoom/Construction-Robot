classdef Counstruct_Rob < handle
  
    
    properties
        
        sim_client
        UR5_target_handle
        UR5_tip_handle
        RG2_tip_handle
        simxModelproperty_not_dynamic
        %Proximity_sensor
        workspace_limits =  [[-0.724, -0.276]; [-0.224, 0.224]; [-0.0001, 0.4]]; % x; y; z

        is_testing = false
        test_preset_cases = false
        
        

        vrep
        
        object_handles = [ ]
                
    end
    
    properties (Hidden)
    end
    
    methods
        
        
        function obj = Counstruct_Rob()
            
            % object vrep ~ vrep software
            vrep=remApi('remoteApi');

            % close everything 
            vrep.simxFinish(-1);

            % client to vrep
            obj.sim_client =vrep.simxStart('127.0.0.1', 19997, true,true, 5000, 5);
            % (server,port,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs) 

            if obj.sim_client == -1
                disp('Failed to connect to simulation (V-REP remote API server). Exiting.')
                exit()
            else
                disp('Connected to simulation')
                obj.vrep = vrep;
            end
            [ret,obj.UR5_tip_handle]=vrep.simxGetObjectHandle(obj.sim_client,'UR5_tip', vrep.simx_opmode_blocking);
            
            % camera settings
            %obj.setup_sim_camera(vrep)
            % Add objects to simulation environment
            %%obj.add_objects(vrep)
            %%for i=1:100
                %%[ret,obj.object_handles(end+1)]=vrep.simxGetObjectHandle(obj.sim_client, ['object' convertStringsToChars(int2str(i))] , vrep.simx_opmode_blocking);
            %%end

        end
        
  
            
        function rotat(self, heightmap_rotation_angle, vrep)
            [sim_ret, gripper_orientation] = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, vrep.simx_opmode_blocking);
            tool_rotation_angle=(mod(heightmap_rotation_angle, pi) ) - pi/2;
            if (tool_rotation_angle - gripper_orientation(2) > 0)
                rotation_step = 0.3;
            else
                rotation_step = -0.3;
            end
            %rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
            num_rotation_steps = floor( (tool_rotation_angle - gripper_orientation(2))/rotation_step )

            % Simultaneously move and rotate gripper
            for i =1:max(abs(num_rotation_steps))
                step_iter = i -1;
                ori = [pi/2, gripper_orientation(2) + rotation_step*min(step_iter,num_rotation_steps), pi/2];
                vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, ori, vrep.simx_opmode_blocking);
            end
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, [pi/2, tool_rotation_angle, pi/2], vrep.simx_opmode_blocking)
        end
        function move_to(self, tool_position, vrep)
             % sim_ret, UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'UR5_target',vrep.simx_opmode_blocking)
            [sim_ret, UR5_target_position] = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle,-1, vrep.simx_opmode_blocking);

            move_direction = tool_position - UR5_target_position;            
            move_magnitude = norm(move_direction)
            move_step = 0.02*move_direction/move_magnitude;
            num_move_steps = floor(move_magnitude/0.02);

            for i = 1:num_move_steps
                pos = UR5_target_position + move_step;
                vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle,-1, pos, vrep.simx_opmode_blocking);
                [sim_ret, UR5_target_position] = vrep.simxGetObjectPosition(self.sim_client,self.UR5_target_handle,-1,vrep.simx_opmode_blocking);
            end
            
            vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle,-1, tool_position, vrep.simx_opmode_blocking);
 
        end
        

        function gripper_fully_closed =  close_gripper(self, vrep)
            gripper_motor_velocity = -0.5;
            gripper_motor_force = 100;
            [sim_ret, RG2_gripper_handle] = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint', vrep.simx_opmode_blocking);
            [sim_ret, gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking);
            vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity, vrep.simx_opmode_blocking);
            gripper_fully_closed = false;
            while (gripper_joint_position > 0.017) % Block until gripper is fully closed
                [sim_ret, new_gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking);
                % disp(gripper_joint_position)
                if new_gripper_joint_position >= gripper_joint_position
                    return 
                end
                gripper_joint_position = new_gripper_joint_position;
            
            end
            gripper_fully_closed = true;

            
        end
        
        function open_gripper(self, vrep)
            gripper_motor_velocity = 0.5;
            gripper_motor_force = 20;
            [sim_ret, RG2_gripper_handle] = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint', vrep.simx_opmode_blocking);
            [sim_ret, gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking);
            vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity, vrep.simx_opmode_blocking);
            while (gripper_joint_position < 0.025) % Block until gripper is fully open
                [sim_ret, gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking)
            end
            
        end
        
        function check_sim(self)

        % Check if simulation is stable by checking if gripper is within workspace
        [sim_ret, gripper_position] = self.vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, self.vrep.simx_opmode_blocking);
        sim_ok = (gripper_position(1) > self.workspace_limits(1,1) - 0.1) & ( gripper_position(1) < self.workspace_limits(1,2) + 0.1) & ( gripper_position(2) > self.workspace_limits(2,1) - 0.1 ) & ( gripper_position(2) < self.workspace_limits(2,2) + 0.1) & ( gripper_position(3) > self.workspace_limits(3,1) ) & ( gripper_position(3) < self.workspace_limits(3,2)  )
        if ~ sim_ok
            disp('Simulation unstable. Restarting environment.')
            self.restart_sim(self.vrep)
            self.add_objects()
        end
        
        end
        
        function outputArg = method1(obj,inputArg)
            outputArg = obj.Property1 + inputArg;
        end
        
    end % methods
    
end

