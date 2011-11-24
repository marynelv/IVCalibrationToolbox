% Rotation-only UKF
clc
close all

%% UKF parameters
ukf_alpha = 0.1;
ukf_beta = 2;


%% x: state vector
% q_w_i = x(1:4);           % IMU rotation quaternion in the world frame [w x y z]'
%% P: state covariance matrix 3-by-3

%% u: process inputs
% u = w_m(1:3, i);          % Measured IMU rotational velocity

%% n: process noise

%% Q: process noise covariance matrix
Q = eye(3) * 0.002 * 300 * pi/180; % rad/s

%% z: measurements
% See section 4.3 Measurement Model on page 11
% z is a 2n-by-1 column vector of observed pixel coordinates in the
% form [x1 y1 ... xn yn]' where n is the number of 3D feature points

%% R: measurement noise covariance matrix
% The associated block-daigonal covariance matrix of z
% R = diag(R1 ... Rn)

%% Starting index
i = 1;
j = 1;
nowTime = -0.01;

%% Initial estimate
% x(1:3,1) = p_w(:,i); % Let's make this easy and set it to the ground truth location
expected_rad_error = .001 * pi / 180;
init_rad_error = 0.2* expected_rad_error;
rand_quat = matrix2quaternion(rotx(init_rad_error)*roty(init_rad_error)*rotz(init_rad_error));
% x(1:4,1) = quaternionproduct(q_w_i(:,i), rand_quat);
%x(1:4,1) = q_w_i(:,1);
%x(1:4,1)=q_i_c;
clear x;
x=zeros(4,1);
x(1:4,1)=q_i_c;
%x(1:4,1)=rand_quat;
P = expected_rad_error * eye(3);



%% Initialize storage matrices
numCamMeasurements = size(observed_pts_c, 2);
numImuMeasurements = length(imuData);
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
accumQuat = NaN * ones(4,numPoses);
distanceError = zeros(1, numPoses);


%% Begin Kalman filter

count = 1;
while (i <= numImuMeasurements && j <= numCamMeasurements )
    
    % Read the timestamp for the next data input
    imuTime = imuData(i,3);
    camTime = camData(j,3);
    
    if (imuTime < camTime)
        %% Prediction step
        pastTime = nowTime;
        nowTime = imuTime;
%         dt = nowTime - pastTime;
%         
%         u = gyro_i_measured(1:3, i);
%         
%         process_params{1} = u;
%         process_params{2} = dt;
%         process_params{3} = x(1:4);
%         process_handle = @processModelQuat;
%         
%         x_se = [0 0 0]'; % State error vector in MRP
%         [x_se, P, process_out] = predictUFK(x_se, process_handle, process_params, P, Q, ukf_alpha, ukf_beta);
%         mean_q = process_out{1};
%         
%         mrp_error = x_se(1:3);
%         % Convert MRP error vector to quaternion error
%         norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
%         dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
%         
%         q_error = [ dq0;
%             bsxfun(@times,(1+dq0),mrp_error)];
%         
%         quat_new = quaternionproduct(q_error, mean_q)';
%         x(1:4) = quat_new./norm(quat_new);
%     P
    i = i + 1;
    else
        %% Correction Step
        
        % Perform correction step
        z = noisy_observed_pts_c(:,j);
        %         R = reshape(camData(j,11:46), 6, 6);
        %         R = std_pixel_noise^2 * eye(length(z));
        R = 0.1^2 * eye(length(z));
        
        x_se = [0 0 0]'; % State error vector in MRP
        ukf_N = length(x_se);
        
        p_IMU_camera = repmat(p_i_c, 1, 2*ukf_N+1);
        p_world_IMU = repmat(p_w(:,j), 1, 2*ukf_N+1);
        q_world_IMU = repmat(q_w_i(:,j), 1, 2*ukf_N+1);
        p_world_pts = pts_w(1:3, :);
        
%         K = eye(3);
        obs_params{1} = x(1:4);
        obs_params{2} = p_world_IMU;
        obs_params{3} = p_IMU_camera;
        obs_params{4} = q_world_IMU;
        obs_params{5} = p_world_pts;
        obs_params{6} = K;
        obs_handle = @measurementModelIMUCameraRotation;
        
%         [M,Pp,K,MU,S,LH] = UKF_UPDATE1(x_se,P,z,obs_handle,R,obs_params,ukf_alpha,ukf_beta);
        [ x_se, P ] = correctUKF( x_se, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta );
        
%           mrp_error = M(1:3);
        mrp_error = x_se(1:3);
        % Convert MRP error vector to quaternion error
        norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
        dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
        
        q_error = [ dq0;
            bsxfun(@times,(1+dq0),mrp_error)];
%         q_error = q_error./norm(q_error);
        quat_new = quaternionproduct(q_error, x(1:4))';
        quat_new = quat_new ./ norm(quat_new);
        x(1:4) = quat_new;
        
        j = j + 1;
    end
    
    %% Distance error
    distanceError(1,count) = findQuaternionError(x, q_i_c);
    
    %% Plot
    accumQuat(:,count) = x;
    %     accumOrient(:,count) = cmatrix(x(1:3))*[0 0 1]';
    count = count + 1;
    x;
    
    if mod(count, 10) == 1
        figure(1)
        clf
        
        %         subplot(2,1,1);
        %         plot3(accumPoses(1,1:count-1), accumPoses(2,1:count-1), accumPoses(3,1:count-1),'.');
        %         hold on;
        %         plot3(p_w(1,1:i), p_w(2,1:i), p_w(3,1:i), 'g');
        % %         hold on;
        % %         plot3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r.');
        %         axis equal
        %         axis vis3d
        
        
        subplot(2,1,1);
        plotQuaternion( x ); hold on;
        plotQuaternion( q_w_i(:,i) ); 
        axis vis3d;
        hold off;
        
        subplot(2,1,2);
        plot(1:count,distanceError(1:count));
        maxErr = max(distanceError);
        axis([0 numPoses 0 maxErr]);
        xlabel('Time');
        ylabel('Distance to ground truth');
        title('Squared Error');
        
    end
end





