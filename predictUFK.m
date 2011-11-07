function [ xk1, Pk1 ] = predictUFK( x, u, dt, P, ukf_N, ukf_alpha, ukf_beta )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

[sigmaPoints,Weightsm,Weightsc] = calculateSigmaPoints(x, P, ukf_N, ukf_alpha, ukf_beta);

% Eq 70
sigmaPointsk1 =  processModelTranslation(sigmaPoints, u, dt);

% Eq 71
xk1 = sum( bsxfun(@times, sigmaPointsk1, Weightsm'), 2);

% Eq 72
stateDiff = bsxfun(@minus, sigmaPoints, xk1);
Pk1 = zeros(size(P));
for i = 1:size(stateDiff,2) % Not sure how to do this without a for loop
    Pk1 = Pk1 + Weightsc(i)*stateDiff(:,i)*stateDiff(:,i)';
end

end
