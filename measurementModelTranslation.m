function z=measurementModelTranslation(p_world_IMU, p_IMU_camera, q_world_IMU, q_IMU_camera, p_world_pts, K)

% z=measurementModelTranslation(p_world_IMU, p_IMU_camera, q_world_IMU, q_IMU_camera, p_world_pts, K)
%
% p_world_IMU: 3x1 vector
% p_IMU_camera: 3x1 vector
% q_world_IMU: 4x1 vector (unit quat)
% q_IMU_camera: 4x1 vector (unit quat)
% p_world_pts: 3xP points
% K: 3x3 matrix
%
% z: 2xP points

C_q_world_IMU=quaternionToMatrix(q_world_IMU);
C_q_IMU_camera=quaternionToMatrix(q_IMU_camera);

p_IMU_pts=C_q_world_IMU*bsxfun(@minus,p_world_pts,p_world_IMU);
p_camera_pts=C_q_IMU_camera*bsxfun(@minus,p_IMU_pts,p_IMU_camera);

p_camera_pts_proj=K*p_camera_pts;
z=bsxfun(@rdivide,p_camera_pts_proj(1:2,:),p_camera_pts_proj(3,:));


end