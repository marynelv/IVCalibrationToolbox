function p_world_IMU_next=processModelTranslation(p_world_IMU, v_world, timestep)

%
% p_world_IMU_next=processModelTranslation(p_world_IMU,v_world,timestep);
%
% p_world_IMU: 3x1 vector (or 3x2N+1 matrix)
% v_world    : 3x1 vector (or 3x2N+1 matrix)
% timestep   : scalar
% p_world_IMU_next : 3x1 vector (or 3x2N+1 matrix)

p_world_IMU_next=bsxfun(@plus,p_world_IMU,bsxfun(@times,v_world,timestep));

end