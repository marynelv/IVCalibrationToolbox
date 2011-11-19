function [ angle_rad ] = findQuaternionError( q1, q2 )
%findQuaternionError Smallest angle between any two quaternions [w x y z]
% q1 and q2 both specific a rotation from a global frame to local frame 1
% and 2 respectively. This functions will find the absolute value of the
% smallest angle in radians between the two frames.


q_diff = quaternionproduct(quaternionconjugate(q1), q2);

angle_rad = abs(2*acos(q_diff(1)));


end

