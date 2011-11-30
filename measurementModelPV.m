% z = measurementModelPQV(x, params)
% x: 10x1 vector or 10x2N+1 matrix with [pos; orientation(mrp); vel]
% params: 
%   1 - current orientation estimate (quaternion)
%   2 - translation from the IMU to the camera (p_IMU_camera)
%   3 - rotation from the IMU to the camera (quaternion q_IMU_camera)
%   4 - 3xP landmarks in the world
%   5 - 3x3 intrinsic camera parameters (K)
function z = measurementModelPV(x, params)

q_w_i = params{1};
p_IMU_camera = params{2};
q_IMU_camera = params{3};
p_world_pts = params{4};
K = params{5};

p_w = x(1:3,:);
% de_world_IMU = x(4:6,:);
% v_w = x(7:9,:);           % it's part of the state vector, but we don't need it

P = size(p_world_pts,2);    % # landmarks
N = size(p_w,2);            % # states to process

z = zeros(2*P, N);

for i=1:N
   
    % handle orientation
    C_q_world_IMU = quaternion2matrix(q_w_i);
    C_q_IMU_camera=quaternion2matrix(q_IMU_camera(:,i));
    
    p_IMU_pts=C_q_world_IMU(1:3, 1:3)'*bsxfun(@minus,p_world_pts,p_w(:,i));
    p_camera_pts=C_q_IMU_camera(1:3, 1:3)'*bsxfun(@minus,p_IMU_pts, p_IMU_camera(:,i));
    
    p_camera_pts_proj=K*p_camera_pts;
    zi=bsxfun(@rdivide,p_camera_pts_proj(1:2,:),p_camera_pts_proj(3,:));
    
    z(:,i)=zi(:);
    
end

end