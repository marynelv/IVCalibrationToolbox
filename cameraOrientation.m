% Find camera orientation given its position, velocity and the mean
% landmark location
% q = cameraOrientation(p_c_w, up, lookAtPoint)
%   p_c_w       3x1 - camera position in the world
%   up          3x1 - (normalized) up vector
%   lookAtPoint 3x1 - look at point in the world
%   q        4x1 - quaternion
function q = cameraOrientation(p_c_w, up, lookAtPoint)

% build camera frame
z = p_c_w - lookAtPoint;
y = up;
x = cross(y,z);
y = cross(z,x);

z = z/norm(z);
y = y/norm(y);
x = x/norm(x);

% rotation matrix from world frame to camera
R = [x y z];

assert(abs(det(R) - 1) < 1e-6);

% corresponding quaternion
q = rotation2quaternion(R);

