% Quaternion derivative
% dqdt=quaternionDerivative(q, t)
% q: 4xn
% t: 1xn
% dqdt: 4x(n-1)
function dqdt=quaternionDerivative(q, t)

N = size(t, 2);
dqdt = zeros(4, N-1);
for i=2:N
    w = [angleChange(q(:,i-1), q(:,i), t(i)-t(i-1)); 0];
    dqdt(:,i-1) = 0.5*quaternionproduct(w', q(:,i-1));
end

end

% w = angleChange(q1, q2, deltaT)
%   q1  4x1 - quaternion 1
%   q1  4x1 - quaternion 2
%   deltaT  - time step
%   w   4x1 - angle change from q1 to q2
function w = angleChange(q1, q2, deltaT)

q = quaternionproduct(q2', quaternionconjugate(q1'));
q = q/norm(q);

k = q(2:4)';     % organized as [w x y z]'
sinThetaDiv2 = norm(k);
cosThetaDiv2 = q(1);

if (sinThetaDiv2 > 1e-12),
    theta = 2*atan2(cosThetaDiv2, sinThetaDiv2);
    k = k/sinThetaDiv2;
    w = k*theta/deltaT;
else
    w = [0 0 0]';
end

end



