% Position-only UKF

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
x(1:3,1) = p_w(:,i); % Let's make this easy and set it to the ground truth location
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
    
    if (imuTime < camTime)
        %% Prediction step
        pastTime = nowTime;
        nowTime = imuTime;
        dt = nowTime - pastTime;
        
        u = noisy_v_w(1:3, i);   
        

        [x P] = predictUFK(x, u, dt, P, Q, ukf_alpha, ukf_beta);
        
        i = i + 1;        
    else        
        %% Correction Step

        % Perform correction step
        z = noisy_observed_pts_c(:,j);
%         R = reshape(camData(j,11:46), 6, 6);
%         R = eye(length(z));
        R = std_pixel_noise^2 * eye(length(z));
        
        
        [sigmaPoints,Weightsm,Weightsc] = calculateSigmaPoints(x, P, ukf_N, ukf_alpha, ukf_beta);
        
        % Eq 73
        numSigmaPoints = size(sigmaPoints, 2);
        p_IMU_camera = repmat(p_i_c, 1, numSigmaPoints);
        q_world_IMU = repmat(q_w_i(:,j), 1, numSigmaPoints);
        q_IMU_camera = repmat(q_i_c, 1, numSigmaPoints);
        p_world_pts = pts_w(1:3, :);
        K = eye(3);
        gamma = measurementModelTranslation(sigmaPoints, p_IMU_camera, q_world_IMU, q_IMU_camera, p_world_pts, K);
        
        % Eq 74
        z_pred = sum( bsxfun(@times, gamma, Weightsm'), 2);
        
        stateDiff = bsxfun(@minus, sigmaPoints, x);
        measurementDiff = bsxfun(@minus, gamma, z_pred);
        Pxz = zeros(length(x), length(z));
        Pzz = zeros(length(z), length(z));
        for k = 1:size(stateDiff,2) % Not sure how to do this without a for loop
            Pxz = Pxz + Weightsc(k)*stateDiff(:,k)*measurementDiff(:,k)';
            Pzz = Pzz + Weightsc(k)*measurementDiff(:,k)*measurementDiff(:,k)';
        end
        
        K = Pxz/(Pzz+R);
        innovation = z - z_pred;
        x_plus = x + K*innovation;
        P_plus = P - K*Pzz*K';
        
        x = x_plus;
        P = P_plus;

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





