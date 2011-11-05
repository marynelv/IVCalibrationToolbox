function p_world_IMU_next=processModelTranslation(p_world_IMU, v_world, timestep)

%
% p_world_IMU_next=processModelTranslation(p_world_IMU,v_world,timestep);
%
% p_world_IMU: 3x1 vector
% v_world    : 3x1 vector
% timestep   : scalar
% p_world_IMU_next : 3x1 vector

p_world_IMU_next=p_world_IMU+bsxfun(@times,v_world,timestep);

end