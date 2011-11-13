% Rotation-only UKF

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
j = 2;
nowTime = -0.01;

%% Initial estimate
% x(1:3,1) = p_w(:,i); % Let's make this easy and set it to the ground truth location
x(1:4,1) = [1 0 0 0]';
P = diag([10 10 10]) * pi / 180;



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
    
    if (imuTime < camTime)
        %% Prediction step
        pastTime = nowTime;
        nowTime = imuTime;
        dt = nowTime - pastTime;
        
        u = gyro_i_measured(1:3, i);   
        
        process_params{1} = u;
        process_params{2} = dt;
        process_params{3} = x(1:4);
        process_handle = @processModelQuat;
        
        x_se = [0 0 0]'; % State error vector in MRP
        [x, P, process_out] = predictUFK(x_se, process_handle, process_params, P, Q, ukf_alpha, ukf_beta);
        mean_q = process_out{1};
        
        mrp_error = x(1:3);
        % Convert MRP error vector to quaternion error
        norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
        dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
        
        q_error = [ dq0;
            bsxfun(@times,(1+dq0),mrp_error)];

        x(1:4) = quaternionproduct(q_error, mean_q)';
        
        i = i + 1;        
    else        
        %% Correction Step

        % Perform correction step
        z = noisy_observed_pts_c(:,j);
%         R = reshape(camData(j,11:46), 6, 6);
%         R = std_pixel_noise^2 * eye(length(z));
        R = 0.1^2 * eye(length(z));
                
        p_IMU_camera = repmat(p_i_c, 1, 2*ukf_N+1);
        q_world_IMU = repmat(q_w_i(:,j), 1, 2*ukf_N+1);
        q_IMU_camera = repmat(q_i_c, 1, 2*ukf_N+1);
        p_world_pts = pts_w(1:3, :);
        K = eye(3);
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
    distanceError(1,count) = norm(x(1:3) - p_w(:,i));
    
    %% Plot
    accumPoses(:,count) = x(1:3);
%     accumOrient(:,count) = cmatrix(x(1:3))*[0 0 1]';
    count = count + 1; 
    x
    
    if mod(count, 10) == 1
        figure(1)
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





