% [x_next] = processModelPQV(xa, params)
% xa: 9x1 vector (or 9x2N+1 matrix) representing augmented state vector
% params:
%   1 - process inputs u = [a_i(1:3, i); w(1:3, i)];
%   2 - time step
%   3 - current orientation estimate (quaternion)
%   4 - gravity in the world
function [x_next] = processModelPV(xa, params)

a_i      = params{1};
timestep = params{2};
q_w_i    = params{3}; % quaternion orientation
gravity  = params{4};

% State vector
position   = xa(1:3,:);  % p_w
velocity   = xa(4:6,:);  % v_w
% Augmented portion of the state vector
noise_acc  = xa(7:9,:);

% Reserve space for result and intermediate computation
numSigmaPoints = size(xa, 2);
x_next = zeros(6, numSigmaPoints);

% Find the true acceleration in the IMU frame by removing the noise and
% gravity from the measurement. Here we are assuming no bias in the accel
% readings
a_i_minus_noise = bsxfun(@minus, a_i, noise_acc);

% compute real acceleration
C_q_world_IMU = quaternion2matrix(q_w_i);
real_accel = C_q_world_IMU(1:3, 1:3)*a_i_minus_noise + ...
                repmat(gravity, 1, numSigmaPoints);    

x_next(1:3,:) = position + velocity.*timestep;
x_next(4:6,:) = velocity + real_accel.*timestep;

end