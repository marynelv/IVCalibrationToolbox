% Position-only UKF



%% x: state vector
% p_w_i = x(1:3);           % IMU position in the world frame
% q_w_i = x(4:7);           % Quaternion [w x y z] IMU rotation in world frame
% v_w = x(8:10);            % Velocity of IMU in the world frame
% bias_gyro = x(11:13);     % IMU gyroscope bias
% bias_accel = x(14:16);    % IMU accelerometer bias
% gravity_w = x(17:19);     % Gravity vector in the world frame
% p_i_c = x(20:22);         % Camera position in the IMU frame
% q_i_c = x(23:26);         % Quaternion [w x y z] camera rotation in IMU frame

%% P: state covariance matrix

%% u: process inputs

%% n: process noise

%% Q: process noise covariance matrix

%% z: measurements

%% R: measurement noise covariance matrix


%% Starting index
i = 1;

%% Initial estimate
x(1:3,1) = p_w(:,i); % Let's make this easy and set it to the ground truth location
P = diag([0.5 0.5 0.5]);




