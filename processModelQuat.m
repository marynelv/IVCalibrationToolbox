function [x_next]=processModelQuat(xa,params)

%
% xa_next=processModelTranslation(xa,v_world,timestep);
%
% xa: 6x1 vector (or 6x2N+1 matrix) -- augmented with noise!
% v_world    : 3x1 vector (or 3x2N+1 matrix)
% timestep   : scalar
% xa_next : 3x1 vector (or 3x2N+1 matrix)

gyro_measurement = params{1};
timestep = params{2};
expected_q_w_i = params{3};

% State vector
mrp_error = xa(1:3,:); % Error of the world to IMU quaterion in MRPs
% Augmented portion of the state vector
noise_gyro = xa(4:6,:);

numSigmaPoints = size(xa, 2);

% Convert MRP error vector to quaternion error for all the sigma points
norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
q_error = [ dq0; bsxfun(@times,(1+dq0),mrp_error)];

% Find the true angular velocity in the IMU frame by removing the noise
% from the measurement. Here we are assuming no bias in the gyro readings
w_i = bsxfun(@minus, gyro_measurement, noise_gyro);

% Preallocate matrices for all the sigma points
sigma_q_w_i = zeros(4, numSigmaPoints);
dq_dt = zeros(4, numSigmaPoints);
sigma_qk1 = zeros(4, numSigmaPoints);
sigma_delta_q_k1 = zeros(4, numSigmaPoints);


for i=1:numSigmaPoints
    sigma_q_w_i(:, i) = quaternionproduct(q_error(:,i), expected_q_w_i)';
    skew_w = skewSymmetric(w_i(:,i));    
    omega_w = [ 0,          -w_i(:,i)';
                w_i(:,i),   -skew_w];
    
    % Quaternion time derivative
    dq_dt(:,i) = 0.5*omega_w*sigma_q_w_i(:,i);
    
    % Propagate the quaternion from k to k+1 using the quaternion time
    % derivative multiplied by the time step
    sigma_qk1(:,i) = sigma_q_w_i(:,i) + timestep * dq_dt(:,i);
    
    % The change in the quaternion at k to all the possible quaternions at
    % k+1. These will be converted back to MRPs and sent out with the
    % updated state vector.
    sigma_delta_q_k1(:,i) = quaternionproduct(sigma_qk1(:,i), quaternionconjugate(sigma_q_w_i(:,1))')';
    
end

% Convert the delta quaternions to MRPs and return them.
sigma_mrp_k1 = bsxfun(@rdivide, sigma_delta_q_k1(2:4,:), (1 + sigma_delta_q_k1(1,:)));
x_next = [ sigma_mrp_k1 ];


end