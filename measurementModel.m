function z_k=measurementModel(x_k, p_world_pts, K)

% x_k: 26x1 state vector or 26 x 2N+1 sigma points
% world_pts: 3xP points
% K: 3x3 matrix
% usetranslationonly: true or false, true if only translation will be used
% z_k: 2Px1 output (first all x coordinates, then all y coordinates)

p_world_IMU=x_k(1:3,:);
p_IMU_camera=x_k(20:22,:);
q_world_IMU=x_k(4:7,:);
q_IMU_camera=x_k(23:26,:);

P=size(p_world_pts,2);
N=size(p_world_IMU,2);

z_k=zeros(2*P, N);

for i=1:N

C_q_world_IMU=quaternion2matrix(q_world_IMU(:,i));
C_q_IMU_camera=quaternion2matrix(q_IMU_camera(:,i));

p_IMU_pts=C_q_world_IMU(1:3, 1:3)'*bsxfun(@minus,p_world_pts,p_world_IMU(:,i));
p_camera_pts=C_q_IMU_camera(1:3, 1:3)'*bsxfun(@minus,p_IMU_pts,p_IMU_camera(:,i));

p_camera_pts_proj=K*p_camera_pts;
zi=bsxfun(@rdivide,p_camera_pts_proj(1:2,:),p_camera_pts_proj(3,:));
%zi=zi';
z_k(:,i)=zi(:);

end

end