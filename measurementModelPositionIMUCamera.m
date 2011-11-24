function z=measurementModelPositionIMUCamera(x, params)


% p_world_IMU: 3x1 vector or 3x2N+1 matrix
% p_IMU_camera: 3x1 vector or 3x2N+1 matrix
% q_world_IMU: 4x1 vector (unit quat) or 4x2N+1 matrix
% q_IMU_camera: 4x1 vector (unit quat) or 4x2N+1 matrix
% p_world_pts: 3xP points
% K: 3x3 matrix
%
% z: 2P x 1 vector or 2P x (2N+1) matrix

q_mean = params{1};
%p_world_IMU = params{2};
q_world_IMU = params{2};
p_world_pts = params{3};
K = params{4};

P=size(p_world_pts,2);
p_world_IMU=x(1:3,:);
p_IMU_camera=x(4:6,:);
de_IMU_camera=x(7:9,:);
N=size(de_IMU_camera,2);

z=zeros(2*P, N);

for i=1:N
    
    de=de_IMU_camera(:,i);
    dq0=(1-norm(de))/(1+norm(de));
    dq=(1+dq0)*de;
    delta_quat = [dq0;dq];
    delta_quat = delta_quat./norm(delta_quat);
    
    Cq=quaternion2matrix(q_mean);
    Cdq=quaternion2matrix(delta_quat);
    
%   C_q_world_IMU=Cdq*Cq;
%   C_q_world_IMU=Cq*Cdq;
    C_q_IMU_camera = quaternion2matrix(quaternionproduct(delta_quat, q_mean));

    C_q_world_IMU=quaternion2matrix(q_world_IMU(:,i));
    
    p_IMU_pts=C_q_world_IMU(1:3, 1:3)'*bsxfun(@minus,p_world_pts,p_world_IMU(:,i));
    p_camera_pts=C_q_IMU_camera(1:3, 1:3)'*bsxfun(@minus,p_IMU_pts, p_IMU_camera(:,i));
    
    p_camera_pts_proj=K*p_camera_pts;
    zi=bsxfun(@rdivide,p_camera_pts_proj(1:2,:),p_camera_pts_proj(3,:));
    %zi=zi';
    z(:,i)=zi(:);
    
end

end