% Compute Kalman gain
% K = kalmanGain(P_xz, P_zz, R)
%   P_xz    NxM - state-measurement cross-covariance matrix
%   P_zz    MxM - predicted measurement covariance matrix
%   R       MxM - measurement covariance matrix    
%   K       NxM - Kalman gain
%
% NOTE: Implementation of equation (77), page 25
function K = kalmanGain(P_xz, P_zz, R)

K = P_xz/(P_zz + R);

end