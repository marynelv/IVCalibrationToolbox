function [ xk1, Pk1 ] = predictUKF( x, process_fun, process_params, P, Q, ukf_alpha, ukf_beta )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Create the augmented state vector and state covariance matrix
x_aug = [x; zeros(size(Q,1), 1)];
P_aug = blkdiag(P, Q);

% The number of sigma points are calculated based on the size of the
% augmented state vector
ukf_N = length(x_aug);

% Calculate ukf_N sigma points
[sigmaPoints,Weightsm,Weightsc] = calculateSigmaPoints(x_aug, P_aug, ukf_N, ukf_alpha, ukf_beta);

% Pass the sigma points through the process model
% Eq 70
sigmaPointsk1 = process_fun(sigmaPoints, process_params);
% Find the new estimate for the state vector based on the weighted sum of
% the processed sigma points
% Eq 71
xk1 = sum( bsxfun(@times, sigmaPointsk1, Weightsm'), 2);

% Calculate the new state covariance matrix
% Eq 72
stateDiff = bsxfun(@minus, sigmaPointsk1(1:length(x), :), xk1);
Pk1 = zeros(size(P));
for i = 1:size(stateDiff,2) % Not sure how to do this without a for loop
    Pk1 = Pk1 + Weightsc(i)*stateDiff(:,i)*stateDiff(:,i)';
end

end

