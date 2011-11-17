function [x_next, process_out]=processModelQuat(xa,params)

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


% p_world = xa(1:3);
% mrp_error = xa(4:6);
% v_world = xa(7:9);
% gyro_bias = xa(10:12);
% accel_bias = xa(13:15);
%
% noise_gyro_walk = xa(16:18);
% noise_accel_walk = xa(19:21);
% noise_gyro = xa(22:24);
% noise_accel = xa(25:27);

mrp_error = xa(1:3,:);
noise_gyro = xa(4:6,:);

% Convert MRP error vector to quaternion error
norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);

q_error = [ dq0;
    bsxfun(@times,(1+dq0),mrp_error)];

numSigmaPoints = size(xa, 2);

w_i = bsxfun(@minus, gyro_measurement, noise_gyro);


% This should be the zero error or identity quaternion
if (~all(q_error(:,1) == [1 0 0 0]'))
    keyboard
end

sigma_q_w_i = zeros(4, numSigmaPoints);
dq_dt = zeros(4, numSigmaPoints);
sigma_qk1 = zeros(4, numSigmaPoints);
sigma_error_q_k1 = zeros(4, numSigmaPoints);


for i=1:numSigmaPoints
    sigma_q_w_i(:, i) = quaternionproduct(q_error(:,i), expected_q_w_i)';
    
    skew_w = [  0           -w_i(3,i)   w_i(2,i);
        w_i(3,i)     0          -w_i(1,i);
        -w_i(2,i)    w_i(1,i)     0];
    
    omega_w = [ 0,     -w_i(:,i)';
        w_i(:,i),   -skew_w];
    
    dq_dt(:,i) = 0.5*omega_w*sigma_q_w_i(:,i);
    
    sigma_qk1(:,i) = sigma_q_w_i(:,i) + timestep * dq_dt(:,i);
    
    sigma_error_q_k1(:,i) = quaternionproduct(sigma_qk1(:,i), quaternionconjugate(sigma_qk1(:,1))')';
    
end

sigma_mrp_k1 = bsxfun(@rdivide, sigma_error_q_k1(2:4,:), (1 + sigma_error_q_k1(1,:)));

x_next = [ sigma_mrp_k1 ];

% Send back the mean quaternion at the next time step
process_out{1} = sigma_qk1(:,1);

end