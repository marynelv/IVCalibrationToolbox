% z = measurementModelPQVTRGBB(x, params)
% x: 10x1 vector or 10x2N+1 matrix with [pos; orientation(mrp); vel]
% params: 
%   1 - current orientation estimate (quaternion)
%   2 - current orientation IMU-cam (quaternion
%   3 - 3xP landmarks in the world
%   4 - 3x3 intrinsic camera parameters (K)
function z = measurementModelPQVTRGBB(x, params)

q_w_i = params{1};
q_i_c = params{2};    
p_world_pts = params{3};
K = params{4};

p_w = x(1:3,:);
de_world_IMU = x(4:6,:);
p_IMU_camera=x(10:12,:);
de_IMU_camera=x(13:15,:);

P = size(p_world_pts,2);    % # landmarks
N = size(p_w,2);            % # states to process

z = zeros(2*P, N);

for i=1:N
   
    % handle orientation
    de=de_world_IMU(:,i);
    dq0=(1-norm(de))/(1+norm(de));
    dq=(1+dq0)*de;
    delta_quat = [dq0;dq];
    delta_quat = delta_quat./norm(delta_quat);
    C_q_world_IMU = quaternion2matrix(quaternionproduct(delta_quat, q_w_i));

    de=de_IMU_camera(:,i);
    dq0=(1-norm(de))/(1+norm(de));
    dq=(1+dq0)*de;
    delta_quat = [dq0;dq];
    delta_quat = delta_quat./norm(delta_quat);
    C_q_IMU_camera=quaternion2matrix(quaternionproduct(delta_quat,q_i_c));
    
    p_IMU_pts=C_q_world_IMU(1:3, 1:3)'*bsxfun(@minus,p_world_pts,p_w(:,i));
    p_camera_pts=C_q_IMU_camera(1:3, 1:3)'*bsxfun(@minus,p_IMU_pts, p_IMU_camera(:,i));
    
    p_camera_pts_proj=K*p_camera_pts;
    zi=bsxfun(@rdivide,p_camera_pts_proj(1:2,:),p_camera_pts_proj(3,:));
    
    z(:,i)=zi(:);
    
end

end