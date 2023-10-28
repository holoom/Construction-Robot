addpath('utils');
rob = Counstruct_Rob();
shifts=rotation_matrx()
[ret,Proximity_sensor]=rob.vrep.simxGetObjectHandle(rob.sim_client,'Proximity_sensor' , rob.vrep.simx_opmode_blocking);
defult_pos=[-0.575,0.45,0.4]
rob.move_to(defult_pos,rob.vrep);
j=-1
%shifts=[[0 0 0];[0,-0.1,0];[-0.075,-0.175,pi/2];[-0.175,-0.175,pi/2];[-0.245,-0.105,0];[-0.245,-0.005,0];[-0.18,0.075,pi/2];[-0.08,0.075,pi/2]];
%shifts=[shifts;shifts]
%for i=1:4
    %shifts=[shifts;shifts]
%end
z=-0.005;
idx=0;
while 1
[res,p_distance,detected_point]=rob.vrep.simxReadProximitySensor(rob.sim_client,Proximity_sensor, rob.vrep.simx_opmode_blocking);

if p_distance>0
    %Getting the position and the orientation of the block
    idx=idx+1;
    if mod(idx,8)==1
        z=z+0.025;
    end
    if j==-1
    [ret,rob.object_handles(end+1)]=rob.vrep.simxGetObjectHandle(rob.sim_client, 'Cuboid' , rob.vrep.simx_opmode_blocking);
    else
    [ret,rob.object_handles(end+1)]=rob.vrep.simxGetObjectHandle(rob.sim_client, ['Cuboid' convertStringsToChars(int2str(j))] , rob.vrep.simx_opmode_blocking);
    end
    [res, pos] = rob.vrep.simxGetObjectPosition(rob.sim_client, rob.object_handles(end), -1, rob.vrep.simx_opmode_blocking);
    [res, ori] = rob.vrep.simxGetObjectOrientation(rob.sim_client, rob.object_handles(end), -1, rob.vrep.simx_opmode_blocking);
   % Getting the posiyion and the orientation of the Gripper
     [res, posg] = rob.vrep.simxGetObjectPosition(rob.sim_client, rob.UR5_tip_handle, -1, rob.vrep.simx_opmode_blocking);
     [res, orig] = rob.vrep.simxGetObjectOrientation(rob.sim_client, rob.UR5_tip_handle, -1, rob.vrep.simx_opmode_blocking);
   %Rotate the gripper in the same orientation as the block
     rotz=ori(3)+pi/2
     rob.rotat(rotz, rob.vrep);
   %open the gripper and grasp then close it
     rob.open_gripper(rob.vrep);
     rob.move_to(pos,rob.vrep);
     gripper_full_closed = rob.close_gripper(rob.vrep);
     while gripper_full_closed==0
         gripper_full_closed = rob.close_gripper(rob.vrep);
     end
   %move the object to dessired position
     pos1=[pos(1),pos(2),pos(3)+0.25]
     pos2=[-0.355+shifts(j+2,1) 0.175+shifts(j+2,2) pos(3)+0.25];
     pos3=[pos2(1) pos2(2) z];
     desired_poses=[pos1;pos2;pos3];
     for i=1:length(desired_poses)
         rob.move_to(desired_poses(i,:),rob.vrep);
         n=1;
         o=1;
         if i==2
             rob.rotat(shifts(j+2,3), rob.vrep);
         end
         if i==3
           while n>0.07 | o>0.07
             rob.move_to(desired_poses(i,:),rob.vrep);
             [res, ori] = rob.vrep.simxGetObjectOrientation(rob.sim_client, rob.object_handles(end), -1, rob.vrep.simx_opmode_blocking);
             [res, pos] = rob.vrep.simxGetObjectPosition(rob.sim_client, rob.object_handles(end), -1, rob.vrep.simx_opmode_blocking);
             e=[pos(1)-pos3(1) pos(2)-pos3(2)];
             o=abs(shifts(j+2,3)-ori(3)-pi/2);
             n=norm(e);
           end
         end
     end
     %Apply Pid controller
     %return back
     rob.open_gripper(rob.vrep)
     desired_poses=[pos2;defult_pos];
     [r,c]=size(desired_poses)
     for i=1:r
         rob.move_to(desired_poses(i,:),rob.vrep);
     end
     j=j+1
     if idx==255
         break
     end
end
end



