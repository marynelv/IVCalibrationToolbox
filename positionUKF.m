% Position-only UKF

%% UKF parameters
ukf_alpha = 1;
ukf_beta = 2;
ukf_N = 3;


%% x: state vector
% p_w_i = x(1:3);           % IMU position in the world frame


%% P: state covariance matrix

%% u: process inputs
% u = v_w(1:3, i);          % IMU velocity measured in the world frame as
% reported by the simulator

%% n: process noise

%% Q: process noise covariance matrix

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
x(1:3,1) = p_w(:,i); % Let's make this easy and set it to the ground truth location
P = diag([0.5 0.5 0.5]);



%% Initialize storage matrices
numCamMeasurements = size(observed_pts_c, 2);
numImuMeasurements = length(imuData);
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
accumOrient = NaN * ones(3,numPoses);



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
        dt = nowTime - pastTime;
        
        u = v_w(1:3, i);   
        
        [x P] = predictUFK(x, u, dt, P, ukf_N, ukf_alpha, ukf_beta);
        
        i = i + 1;        
    else        
        %% Correction Step

        % Perform correction step
        z = observed_pts_c(:,j);
%         R = reshape(camData(j,11:46), 6, 6);
%         R = eye(length(z));
        R = 1e-6*eye(length(z));
        
        
        [sigmaPoints,Weightsm,Weightsc] = calculateSigmaPoints(x, P, ukf_N, ukf_alpha, ukf_beta);
        numSigmaPoints = size(sigmaPoints, 2);
        p_IMU_camera = repmat(p_i_c, 1, numSigmaPoints);
        q_world_IMU = repmat(q_w_i(:,j), 1, numSigmaPoints);
        q_IMU_camera = repmat(q_i_c, 1, numSigmaPoints);
        p_world_pts = pts_w(1:3, :);
        K = eye(3);
        gamma = measurementModelTranslation(sigmaPoints, p_IMU_camera, q_world_IMU, q_IMU_camera, p_world_pts, K);
        
        z_pred = sum( bsxfun(@times, gamma, Weightsm'), 2);
        
        
%         [x, P] = updateEKF(x, P, z, R, global_points);        
        j = j + 1;
    end
            
    %% Plot
    accumPoses(:,count) = x(1:3);
%     accumOrient(:,count) = cmatrix(x(1:3))*[0 0 1]';
    count = count + 1; 
    x
end





