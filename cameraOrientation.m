% Find camera orientation given its position, velocity and the mean
% landmark location
% q = cameraOrientation(p_c_w, v_c_w, lookAtPoint)
%   p_c_w    3x1 - camera position in the world
%   v_c_w    3x1 - camera velocity in the world
function q = cameraOrientation(p_c_w, v_c_w, lookAtPoint)

% build camera frame
z = lookAtPoint - p_c_w;
z = lookAt/norm(lookAt);
x = v_c_w/norm(v_c_w);
y = cross(z,x);

% rotation matrix from world frame to camera
R = [x y z]';

% corresponding quaternion
q = rotation2quaternion(R);
