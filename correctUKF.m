function [ x_plus, P_plus ] = correctUKF( x, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

ukf_N = length(x);

[sigmaPoints,Weightsm,Weightsc] = calculateSigmaPoints(x, P, ukf_N, ukf_alpha, ukf_beta);

% Eq 73
gamma = obs_handle(sigmaPoints, obs_params);

% Eq 74
z_pred = sum( bsxfun(@times, gamma, Weightsm'), 2);

stateDiff = bsxfun(@minus, sigmaPoints, x);
measurementDiff = bsxfun(@minus, gamma, z_pred);
Pxz = zeros(length(x), length(z));
Pzz = zeros(length(z), length(z));
for k = 1:size(stateDiff,2) % Not sure how to do this without a for loop
    Pxz = Pxz + Weightsc(k)*stateDiff(:,k)*measurementDiff(:,k)';
    Pzz = Pzz + Weightsc(k)*measurementDiff(:,k)*measurementDiff(:,k)';
end

K = Pxz/(Pzz+R);
innovation = z - z_pred;
x_plus = x + K*innovation;
P_plus = P - K*Pzz*K';

end

