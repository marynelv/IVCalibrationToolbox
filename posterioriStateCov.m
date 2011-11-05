% Compute the a posteriori state vector 
% X_plus = posterioriStateVector(X_minus, K, Z, Z_predicted
%   X_minus     Nx1 - priori state vector    
%   K           NxM - Kalman gain
%   Z           Mx1 - measurement vector
%   Z_pred      Mx1 - predicted measurement vector
%   P_minus     NxN - prior covariance matrix
%   P_zz        MxM - predicted measurement covariance matrix
%   X_plus      Nx1 - posteriori state vector
%   P_plus      NxN - posteriori covariance matrix
%
% The measurements Z and Z_pred must have their values ordered by
% component. For the camera projection case, each vector should have all
% x-values first and all y-values second. This means that for n landmarks,
% the size of Z and Z_pred would be (nx2)x1.
% 
% NOTE: Implementation of equations (78) and (79), page 25
function [X_plus, P_plus] = ...
    posterioriStateCov(X_minus, K, Z, Z_pred, P_minus, P_zz)

X_plus = X_minus + K(Z - Z_pred);
P_plus = P_minus - K*P_zz*K';

end