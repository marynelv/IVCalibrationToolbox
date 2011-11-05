function [sigmaPoints,Weightsm,Weightsc]=calculateSigmaPoints(x_ahat_plus, P_a_plus, N, alpha, beta)

%
% [sigmaPoints,Weightsm,Weightsc]=calculateSigmaPoints(x_ahat_plus, P_a_plus);
%
% x_ahat_plus: mx1 augmented state vector (m is number of augmented states)
% P_a_plus: mxm augmented covariance matrix
%
% sigmaPoints: m x 2N+1 matrix of points
% Weightsm, Weightsc: 2N+1 x 1 vector of weights for mean and covariance
%   sigmaPoints(:,1) = x_ahat_plus
%   sigmaPoints(:,2:N+1) = x_ahat_plus+S^j
%   sigmaPoints(:,N+2:2*N+1)=x_ahat_plus-S^j

lambda=alpha*alpha*(N+beta)-N;

m=size(x_ahat_plus,1);
nsigma=2*N+1;

sigmaPoints=zeros(m, nsigma);
Weightsm=zeros(nsigma,1);
Weightsc=zeros(nsigma,1);

S=sqrt(lambda+N)*chol(P_a_plus,'lower');

sigmaPoints(:,1)=x_ahat_plus;
sigmaPoints(:,2:N+1)=bsxfun(@plus,x_ahat_plus,S);
sigmaPoints(:,N+2:2*N+1)=bsxfun(@minus,x_ahat_plus,S);

Weightsm(1)=lambda/(lambda+N);
Weightsc(1)=lambda/(lambda+N)+(1-alpha*alpha+beta);
Weightsm(2:end)=1/(2*(lambda+N));
Weightsc(2:end)=Weightsm(2:end);

end