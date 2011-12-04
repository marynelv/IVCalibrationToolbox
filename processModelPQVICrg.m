% [x_next] = processModelPQV(xa, params)
% xa: 18x1 vector (or 18x2N+1 matrix) representing augmented state vector
% params:
%   1 - process inputs u = [a_i(1:3, i); w(1:3, i)];
%   2 - time step
%   3 - current orientation estimate (quaternion)
%   4 - gravity in the world
function [x_next] = processModelPQVICrg(xa, params)

u        = params{1};
timestep = params{2};
q_w_i    = params{3}; % mean quaternion orientation
%gravity  = params{4};

a_i = u(1:3);
w_i = u(4:6);

% State vector
position   = xa(1:3,:);  % p_w
mrp_error  = xa(4:6,:);  % Error of the world to IMU quaterion in MRPs
velocity   = xa(7:9,:);  % v_w
gravity = xa(16:18,:);

% Augmented portion of the state vector
%if size(xa,1)<=15
%oise_acc  = xa(10:12,:);
%oise_gyro = xa(13:15,:);
%else
    noise_acc=xa(19:21,:);
    noise_gyro=xa(22:24,:);
%end

% Reserve space for result and intermediate computation
numSigmaPoints = size(xa, 2);
x_next = zeros(9, numSigmaPoints);
sigma_q_w_i = zeros(4, numSigmaPoints);
dq_dt = zeros(4, numSigmaPoints);
sigma_qk1 = zeros(4, numSigmaPoints);
sigma_delta_q_k1 = zeros(4, numSigmaPoints);

% Convert MRP error vector to quaternion error for all the sigma points
norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
q_error = [ dq0; bsxfun(@times,(1+dq0),mrp_error)]; % dq

% Find the true angular velocity in the IMU frame by removing the noise
% from the measurement. Here we are assuming no bias in the gyro readings
real_w = bsxfun(@minus, w_i, noise_gyro);

% Find the true acceleration in the IMU frame by removing the noise and
% gravity from the measurement. Here we are assuming no bias in the accel
% readings
real_accel = zeros(3,numSigmaPoints);
a_i_minus_noise = bsxfun(@minus, a_i, noise_acc);

for i=1:numSigmaPoints
   
    % local quaternion error
    dq = q_error(:,i)./norm(q_error(:,i));
    % estimated orientation of the IMU in the world
    sigma_q_w_i(:,i) = quaternionproduct(dq, q_w_i);

    % compute real acceleration
    C_q_world_IMU = quaternion2matrix(sigma_q_w_i(:,i));
    real_accel(:,i) = C_q_world_IMU(1:3, 1:3)*a_i_minus_noise(:,i) - gravity(:,i);    
    
    % propagate quaternion in time
    skew_w = skewSymmetric(real_w(:,i));  
    omega_w = [ 0,             -real_w(:,i)';
                real_w(:,i),   -skew_w];
    dq_dt(:,i) = 0.5*omega_w*sigma_q_w_i(:,i); % quat time derivative
    sigma_qk1(:,i) = sigma_q_w_i(:,i) + timestep * dq_dt(:,i);
    % compute the change in the quaternion at k to all the possible quaternions at
    % k+1. These will be converted back to MRPs and sent out with the
    % updated state vector.
    sigma_delta_q_k1(:,i) = ...
        quaternionproduct(sigma_qk1(:,i), quaternionconjugate(sigma_q_w_i(:,1))')';        
    
end

% Convert the delta quaternions to MRPs and return them.
sigma_mrp_k1 = bsxfun(@rdivide, sigma_delta_q_k1(2:4,:), (1 + sigma_delta_q_k1(1,:)));

x_next(1:3,:) = position + velocity.*timestep;
x_next(4:6,:) = sigma_mrp_k1;
x_next(7:9,:) = velocity + real_accel.*timestep;
if size(xa,1)>15
    x_next=[x_next;xa(10:18,:)];
end

end