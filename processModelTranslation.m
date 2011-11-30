function [x_next, out_params]=processModelTranslation(xa,params)

%
% xa_next=processModelTranslation(xa,v_world,timestep);
%
% xa: 6x1 vector (or 6x2N+1 matrix) -- augmented with noise!
% v_world    : 3x1 vector (or 3x2N+1 matrix)
% timestep   : scalar
% xa_next : 3x1 vector (or 3x2N+1 matrix)

 v_world = params{1};
 timestep = params{2};

%p_world_IMU_next=bsxfun(@plus,p_world_IMU,bsxfun(@times,v_world,timestep));
%x_next=bsxfun(@plus,x,bsxfun(@times,v_world,timestep));

% assume that we get noise as well in x
% remove the noise from the position estimate
x_next=bsxfun(@plus,xa(1:3,:)-xa(4:6,:),bsxfun(@times,v_world,timestep));
out_params = [];
end