% Position-only UKF

clear x;

%% UKF parameters
ukf_alpha = 0.1;
ukf_beta = 2;


%% x: state vector
% p_w_i = x(1:3);           % IMU position in the world frame


%% P: state covariance matrix

%% u: process inputs
% u = v_w(1:3, i);          % IMU velocity measured in the world frame as
% reported by the simulator

%% n: process noise

%% Q: process noise covariance matrix
% Q = std_v_w^2 * eye(3);
Q = 0.1^2 *eye(3);
%Q=0*eye(3);

%% z: measurements
% See section 4.3 Measurement Model on page 11
% z is a 2n-by-1 column vector of observed pixel coordinates in the 
% form [x1 y1 ... xn yn]' where n is the number of 3D feature points

%% R: measurement noise covariance matrix
% The associated block-daigonal covariance matrix of z
% R = diag(R1 ... Rn)

%% Starting index
i = 2;
j = 2;
nowTime = -0.01;

%% Initial estimate
clear x
x(1:3,1) = p_w(:,i); % Let's make this easy and set it to the ground truth location
%x(1:3,1) = [0.4 0.4 0.4]';
P = diag([0.5 0.5 0.5]);



%% Initialize storage matrices
numCamMeasurements = size(observed_pts_c, 2);
numImuMeasurements = length(imuData);
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
accumOrient = NaN * ones(3,numPoses);
distanceError = zeros(1, numPoses);


%% Begin Kalman filter
ukf_N = length(x);

count = 1;
while (i <= numImuMeasurements && j <= numCamMeasurements )

    % Read the timestamp for the next data input
    imuTime = imuData(i,3);
    camTime = camData(j,3);
    
    if (imuTime <= camTime)
        %% Prediction step
        pastTime = nowTime;
        nowTime = imuTime;
        dt = nowTime - pastTime;
        
        u = noisy_v_w(1:3, i);   
        
        process_params{1} = u;
        process_params{2} = dt;
        process_handle = @processModelTranslation;
        [x P] = predictUKF(x, process_handle, process_params, P, Q, ukf_alpha, ukf_beta);
        
        i = i + 1;        
    else        
        %% Correction Step

        % Perform correction step
        z = noisy_observed_pts_c(:,j);
%         R = reshape(camData(j,11:46), 6, 6);
%         R = std_pixel_noise^2 * eye(length(z));
        R = 0.1^2 * eye(length(z));
         
        ukf_N = length(x);        
        
        p_IMU_camera = repmat(p_i_c, 1, 2*ukf_N+1);
        q_world_IMU = repmat(q_w_i(:,j), 1, 2*ukf_N+1);
        q_IMU_camera = repmat(q_i_c, 1, 2*ukf_N+1);
        p_world_pts = pts_w(1:3, :);
        %K = eye(3);
        obs_params{1} = p_IMU_camera;
        obs_params{2} = q_world_IMU;
        obs_params{3} = q_IMU_camera;
        obs_params{4} = p_world_pts;
        obs_params{5} = K;
        
        obs_handle = @measurementModelTranslation;
        
        [ x, P ] = correctUKF( x, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta );
         
         j = j + 1;
    end
            
    %% Distance error
    distanceError(1,count) = norm(x(1:3) - p_w(:,i-1));
    
    %% Plot
    accumPoses(:,count) = x(1:3);
%     accumOrient(:,count) = cmatrix(x(1:3))*[0 0 1]';
    count = count + 1; 
    x
    
    if mod(count, 10) == 1
        %figure 
        clf
        
        subplot(2,1,1);
        plot3(accumPoses(1,1:count-1), accumPoses(2,1:count-1), accumPoses(3,1:count-1),'.');
        hold on;
        plot3(p_w(1,1:i), p_w(2,1:i), p_w(3,1:i), 'g');
%         hold on;
%         plot3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r.');
        axis equal
        axis vis3d
        
        subplot(2,1,2);
        plot(1:count,distanceError(1:count));
        maxErr = max(distanceError);
        axis([0 numPoses 0 maxErr]);
        xlabel('Time');
        ylabel('Distance to ground truth');
        title('Squared Error');
        
    end
end





