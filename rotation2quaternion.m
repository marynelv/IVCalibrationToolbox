% Convert 3x3 rotation matrix to quaternion
% q = rotation2quaternion(R)
%   R       3x3 - 3D rotation matrix
%   q       4x1 - quaternion
function q = rotation2quaternion(R)
q = matrix2quaternion([R zeros(3,1); zeros(1,3) 1]);