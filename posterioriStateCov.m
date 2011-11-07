% Compute the a posteriori state vector and covariance matrix from the
% previous state, previous covariance, the sigma points, their weights and
% measurement vector
% [X_plus, P_plus] = ...
%    posterioriStateCov(X_minus, P_minus, X_sigma, W_c, Gamma, Z_pred, Z, R)
%   X_minus     Nx1 - priori state vector  
%   P_minus     NxN - prior covariance matrix 
%   X_sigma     NxS - S sigma points 
%   W_c         Sx1 - sigma points weights (cov)
%   Gamma       MxS - sigma points through the nonlinear measurement model 
%   Z_pred      Mx1 - predicted measurement vector
%   Z           Mx1 - measurement vector
%   R       MxM - measurement covariance matrix   
%
% The measurements Z and Z_pred must have their values ordered by
% landmark. Each vector should have the x-value and the y-value for the 
% first landmark at the beginning, followed by (x,y) of the second landmark 
% and so on ... This means that for n landmarks, the size of Z and Z_pred 
% would be (nx2)x1.
%
% NOTE: Implementation of equations (75) through (79), page 25
function [X_plus, P_plus] = ...
    posterioriStateCov(X_minus, P_minus, X_sigma, W_c, Gamma, Z_pred, Z, R)

[P_xz, P_zz] = covariances(X_minus,X_sigma,W_c,Gamma,Z_pred);
K = kalmanGain(P_xz,P_zz,R);
[X_plus, P_plus] = posterioriStateCov(X_minus,K,Z,Z_pred,P_minus,P_zz);

end

% Compute the state-measurement cross covariance matrix and the predicted
% measurement covarience matrix
% [P_xz, P_zz] = predicCov(X_minus, X_sigma, W, Gamma, Z_pred)
%   X_minus     Nx1 - priori state vector   
%   X_sigma     NxS - S sigma points 
%   W_c         Sx1 - sigma points weights (cov)
%   Gamma       MxS - sigma points through the nonlinear measurement model 
%   Z_pred      Mx1 - predicted measurement vector
%   P_xz        NxM - state-measurement cross-covariance matrix
%   P_zz        MxM - predicted measurement covariance matrix
%
% NOTE: Implementation of equations (75) and (76)
function [P_xz, P_zz] = ...
    covariances(X_minus, X_sigma, W_c, Gamma, Z_pred)

% number of sigma points
S = size(X_sigma,2);
% measurement vector size
M = size(Z_pred,1);
% state vector size
N = size(X_minus,1);

% subtract sigma stuff from means
GammaZ = Gamma - repmat(Z_pred, 1, S);
XX = X_sigma - repmat(X_minus, 1, S);

% compute covariances
P_xz = zeros(N,M);
P_zz = zeros(M,M);
for s=1:S
    P_xz = P_xz + W_c(s,1)*(XX(:,s)*GammaZ(:,s)');
    P_zz = P_zz + W_c(s,1)*(GammaZ(:,s)*GammaZ(:,s)');
end

end


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

% Compute the a posteriori state vector and covariance
% [X_plus, P_plus] = ...
%       posteriori(X_minus, K, Z, Z_pred, P_minus, P_zz)
%   X_minus     Nx1 - priori state vector    
%   K           NxM - Kalman gain
%   Z           Mx1 - measurement vector
%   Z_pred      Mx1 - predicted measurement vector
%   P_minus     NxN - prior covariance matrix
%   P_zz        MxM - predicted measurement covariance matrix
%   X_plus      Nx1 - posteriori state vector
%   P_plus      NxN - posteriori covariance matrix
% 
% NOTE: Implementation of equations (78) and (79), page 25
function [X_plus, P_plus] = ...
    posteriori(X_minus, K, Z, Z_pred, P_minus, P_zz)

X_plus = X_minus + K(Z - Z_pred);
P_plus = P_minus - K*P_zz*K';

end