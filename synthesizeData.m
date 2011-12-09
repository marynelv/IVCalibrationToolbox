% IMU Simulation
% Notation example:
% T_w_i - 4x4 transformation matrix that represents the IMU coordinate frame in the
% world coordinates and will transform a vector from IMU coordinates to
% world coordinates.

% Coordinate frames:
% w: world frame
% i: IMU frame
% c: camera (right-down-forward) frame

% Variables:
% R: rotation 3-by-3
% T: transform 4-by-4
% q: quaternion 4-by-1
% a: acceleration 3-by-1
% v: velocity 3-by-1
% p: position 3-by-1

%% Clear the workspace
clear
close all
clc
% Seed random number generator so that we have repeatable simulation
% results
% rng(2);

%% Setup parameters: time, body translational acceleration, body rotation velocity, noise characteristics
plotFlag = 1;
timeStep = 0.01;
t = 0:timeStep:10; % Simulation run time and time step
sampling_freq = 1/timeStep;


% a_i = sin(t);
% a_i = rand(3,length(t)) - 0.5;
% a_i = repmat([1.4 1.08 0]', 1, length(t));
% a_i = repmat([0 0.8 0]', 1, length(t));
% a_i = repmat([0.3 0 0]', 1, length(t));
% a_i = bsxfun(@times, [1.4 1.08 0.3]', [sin(2*pi*t/0.9); sin(2*pi*t/0.4 - 0.3); sin(2*pi*t/0.1 - 0.8)]);
% w = bsxfun(@times, [.4 .08 0.23]', [sin(2*pi*t/0.2); sin(2*pi*t/1 - 0.3); sin(2*pi*t/0.5 - 0.8)]);
a_i = bsxfun(@times, [0.3 1.08 -0.5]', [sin(2*pi*t/1); sin(2*pi*t/6 - 0.3); sin(2*pi*t/5 - 0.8)]);
w = bsxfun(@times, [0.1 .2 0.5]', [sin(2*pi*t/0.5); sin(2*pi*t/0.6 - 0.3); sin(2*pi*t/4.5 - 0.8)]);


% w = -pi + 2*pi*rand(3,length(t)); %in rad/second
% w = repmat([.1 .3 0.5]', 1, length(t));
% w = repmat([0 0 1]', 1, length(t));
% w = zeros(3, length(t));

% euler_i_c = [ 10*pi/180 -60*pi/180 132*pi/180 ]; % Euler angle rotation from body (IMU) to camera frame in radians
euler_i_c = [0 0 0]';
p_i_c = [ 0 0 0.3]'; % Translation from IMU to camera frame in meters

std_dev_noise_accel = 200e-6 * 9.81 * sqrt(sampling_freq); % According to data sheet 200 ug/sqrt(hz)
std_dev_bias_accel = 0.0042;

std_dev_noise_gyro = 0.05 * pi / 180 * sqrt(sampling_freq); % 0.05 deg/sec/sqrt(Hz) as stated in gyro data sheet
std_dev_bias_gyro = 1.5340e-04; % Taken from zero slope point of Allan Deviation plot at time = 150s times sqrt(2*sampling_freq/dt)*2

gravity = [0 0 9.81]';
% gravity = [0 0 0]';

% Parameters for 3D visual features
numPoints = 30;
pts_min = [-15 -5 5]';
pts_max = [5 15 5.1]';
% pts_min = [-15 -15 -15]';
% pts_max = [15 15 15]';

std_pixel_noise = 1;
% std_pixel_noise = 0;

std_v_w = 0.1;

image_width = 640;                          % image width
image_height = 480;                         % image height
f = 0.7;                                      % focal length
px = image_width/2; py = image_height/2;    % principal point
K = [f*image_width 0 px; 0 f*image_height py; 0 0 1];                % intrinsic parameters

% K = eye(3);             % intrinsic camera parameters


%% Derivated values from setup parameters
T_i_c = rotx(euler_i_c(1))*roty(euler_i_c(2))*rotz(euler_i_c(3));
T_i_c(1:3,4) = p_i_c;
q_i_c = matrix2quaternion(T_i_c);

pts_w = bsxfun(@plus, pts_min, bsxfun(@times, (pts_max-pts_min), rand(3,numPoints)));
pts_w = [pts_w; ones(1, numPoints)];


%% Initialize data structures
a_w = zeros(3,length(t));
v_w = zeros(3,length(t));
p_w = zeros(3,length(t));

q_w_i = [ones(1,length(t)); zeros(3,length(t))];
% qt = [ones(1,length(t)); zeros(3,length(t))];
x_i = [1 0 0]';
y_i = [0 1 0]';
z_i = [0 0 1]';

x_w_i = repmat([1 0 0]', 1, length(t));
y_w_i = repmat([0 1 0]', 1, length(t));
z_w_i = repmat([0 0 1]', 1, length(t));

T_w_i = cell(length(t),1);

observed_pts_c = NaN * ones(2*numPoints, length(t));

%% Run simulation
for i = 2:length(t)
    dt = t(i) - t(i-1);
    
    % Find rotation matrix
    % T_w_i: expresses the body frame in the world frame
    T_w_i{i-1} = quaternion2matrix(q_w_i(:,i-1));
    T_w_i{i-1}(1:3,4) = p_w(:,i-1);
    
    % Calculate IMU acceleration in the world frame
    a_w(:,i-1) = T_w_i{i-1}(1:3, 1:3)*a_i(:,i-1);
    % Add the expected gravity vector back into the measured acceleration
    a_i(:,i-1) = T_w_i{i-1}(1:3, 1:3)'*(a_w(:,i-1) - gravity);
    
    % Update world velocity
    v_w(:,i) = v_w(:,i-1) + a_w(:,i-1)*dt;
    % Update world position
    %     p_w(:,i) = p_w(:,i-1) + v_w(:,i-1)*dt + 0.5*a_w(:,i-1)*dt^2;
    p_w(:,i) = p_w(:,i-1) + v_w(:,i-1)*dt;

    
%     % Update the world to body quaterion
%     if (norm(w(:,i-1)) == 0)
%         dq = [1 0 0 0]';
%     else
%         axisq = w(:,i-1) ./ norm(w(:,i-1));
%         theta = norm(w(:,i-1))*dt;
%         dq = newquaternion(theta, axisq);
%     end
% 
%     q_w_i(:,i) = quaternionproduct(q_w_i(:,i-1), dq);
% %     q_w_i(:,i) = quaternionproduct(dq, q_w_i(:,i-1));
%     q_w_i(:,i) = q_w_i(:,i) / norm(q_w_i(:,i));
    
    % propagate quaternion in time
    skew_w = skewSymmetric(w(:,i-1));  
    omega_w = [ 0,             -w(:,i)';
                w(:,i),   -skew_w];
            
    dq_dt = 0.5*omega_w*q_w_i(:,i-1); % quat time derivative
    
    q_w_i(:,i) = q_w_i(:,i-1) + dt * dq_dt;
    
    q_w_i(:,i) = q_w_i(:,i) / norm(q_w_i(:,i));
    
    
    %     delta_qt = 0.5*rate2quatderiv(w(:,i-1))*qt(:,i-1)*dt;
    %     new_qt = qt(:,i-1) + delta_qt;
    %
    %     qt(:,i) = new_qt ./ norm(new_qt);
    
    %     qt(:,i) = quaternionproduct(qt(:,i), new_qt ./ norm(new_qt));
    %     vec(:,i) = quat_mult(quat_mult(q_w_i(:,i), [0 0 0 1]'), [q_w_i(1,i); -q_w_i(2:end,i)]);
    
    x_w_i(:,i) = quaternionRotate(q_w_i(:,i), x_i);
    y_w_i(:,i) = quaternionRotate(q_w_i(:,i), y_i);
    z_w_i(:,i) = quaternionRotate(q_w_i(:,i), z_i);
    
end
T_w_i{i} = quaternion2matrix(q_w_i(:,i));
T_w_i{i}(1:3,4) = p_w(:,i);


%% Generate simulated measurements
accel_bias_steps = timeStep*std_dev_bias_accel*randn(size(a_i));
bias_accel = cumsum(accel_bias_steps,2);

noise_accel = std_dev_noise_accel*randn(size(a_i));
total_accel_noise = bias_accel + noise_accel;
accel_i_measured = a_i + total_accel_noise;

gyro_bias_steps = timeStep*std_dev_bias_gyro*randn(size(w));
bias_gyro = cumsum(gyro_bias_steps,2);

noise_gyro = std_dev_noise_accel*randn(size(w));
total_gyro_noise = bias_gyro + noise_gyro;
gyro_i_measured = w + total_gyro_noise;

% Store data in ROS ordering
imuData = zeros(length(t), 31);
imuData(:,3) = t;
imuData(:,17:19) = gyro_i_measured';
imuData(:,29:31) = accel_i_measured';

% figure, plot(imuData(:,17));
% figure, plot(imuData(:,18));
% figure, plot(imuData(:,19));
% figure, plot(imuData(:,29));
% figure, plot(imuData(:,30));
% figure, plot(imuData(:,31));        

%% Observed points

for i = 1:length(t)
  pts_c = T_i_c^-1 * T_w_i{i}^-1 * pts_w;
  pts_c = K*pts_c(1:3, :);
  pixels = [pts_c(1,:)./pts_c(3,:); pts_c(2,:)./pts_c(3,:)];
  observed_pts_c(:,i) = pixels(:);
end

camData = zeros(length(t), 3);
camData(:,3) = t;

%% Create noisy measurements
noisy_v_w = v_w + std_v_w*randn(size(v_w));
noisy_observed_pts_c = observed_pts_c + std_pixel_noise*randn(size(observed_pts_c));


%% Plot static results
% figure, plot(t, a_i(1,:), t, a_i(2,:), t, a_i(3,:));
% figure, plot(t,v_w);
% figure, plot(t,p_w);

% figure, plot3(a_w(1,:), a_w(2,:), a_w(3,:));
% figure, plot3(v_w(1,:), v_w(2,:), v_w(3,:));
% figure, plot3(p_w(1,:), p_w(2,:), p_w(3,:));


p_w_c = zeros(3,length(t));
q_w_c = zeros(4,length(t));
for i = 1:length(t)
    T_w_c = T_w_i{i}*T_i_c;
    p_w_c(:,i) = T_w_c(1:3,4);
    q_w_c(:,i) = matrix2quaternion(T_w_c);
end
pts_center = mean(pts_w, 2);

%% Movie like plot
if plotFlag
    for i = 1:length(t)-1
        
        if (mod(t(i), 1) == 0)          
            figure(9), plot3(p_w(1,:), p_w(2,:), p_w(3,:));
            
            hold on;
            scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r', '.');
            
            curR_w_c = T_w_i{i}*T_i_c;
            cam_coor = curR_w_c*[x_i y_i z_i; ones(1,3)];
            
            hold on;
            plot3([curR_w_c(1,4) cam_coor(1,1)], [curR_w_c(2,4) cam_coor(2,1)], [curR_w_c(3,4) cam_coor(3,1)], 'r');            
            hold on;
            plot3([curR_w_c(1,4) cam_coor(1,2)], [curR_w_c(2,4) cam_coor(2,2)], [curR_w_c(3,4) cam_coor(3,2)], 'g');
            hold on;
            plot3([curR_w_c(1,4) cam_coor(1,3)], [curR_w_c(2,4) cam_coor(2,3)], [curR_w_c(3,4) cam_coor(3,3)], 'b');
            hold off
            axis equal
            xlabel('x');
            ylabel('y');
            zlabel('z');
            refresh
            pause(0.1)
        end
    end
end
