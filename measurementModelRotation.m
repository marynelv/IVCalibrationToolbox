function z=measurementModelRotation(de_world_IMU, params)


% p_world_IMU: 3x1 vector or 3x2N+1 matrix
% p_IMU_camera: 3x1 vector or 3x2N+1 matrix
% q_world_IMU: 4x1 vector (unit quat) or 4x2N+1 matrix
% q_IMU_camera: 4x1 vector (unit quat) or 4x2N+1 matrix
% p_world_pts: 3xP points
% K: 3x3 matrix
%
% z: 2P x 1 vector or 2P x (2N+1) matrix

q_mean = params{1};
p_world_IMU = params{2};
p_IMU_camera = params{3};
q_IMU_camera = params{4};
p_world_pts = params{5};
K = params{6};



P=size(p_world_pts,2);
N=size(p_world_IMU,2);

z=zeros(2*P, N);

for i=1:N
    
    de=de_world_IMU(:,i);
    dq0=(1-norm(de))/(1+norm(de));
    dq=(1+dq0)*de;
    
    Cq=quaternion2matrix(q_mean);
    Cdq=quaternion2matrix([dq0;dq]);
    
    C_q_world_IMU=Cdq*Cq;
    C_q_IMU_camera=quaternion2matrix(q_IMU_camera(:,i));
    
    p_IMU_pts=C_q_world_IMU(1:3, 1:3)'*bsxfun(@minus,p_world_pts,p_world_IMU(:,i));
    p_camera_pts=C_q_IMU_camera(1:3, 1:3)'*bsxfun(@minus,p_IMU_pts, p_IMU_camera(:,i));
    
    p_camera_pts_proj=K*p_camera_pts;
    zi=bsxfun(@rdivide,p_camera_pts_proj(1:2,:),p_camera_pts_proj(3,:));
    %zi=zi';
    z(:,i)=zi(:);
    
end

end