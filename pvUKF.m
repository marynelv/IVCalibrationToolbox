% Position, orientation and velocity UKF
clc
close all

%% UKF parameters
ukf_alpha = 0.1;
ukf_beta = 2;

%% x: state vector
% Composed by IMU position and velocity in the world frame
% [p_w_i; v_w_i] = x(1:6); 
%% P: state covariance matrix 6-by-6

%% u: process inputs
% Composed by measured IMU acceleration and rotational velocity
% u = [accel_i_measured(1:3, i)] 

%% n: process noise

%% Q: process noise covariance matrix
Qacc = eye(3) * 0.1^2;
Q = Qacc;

%% z: measurements
% See section 4.3 Measurement Model on page 11
% z is a 2n-by-1 column vector of observed pixel coordinates in the
% form [x1 y1 ... xn yn]' where n is the number of 3D feature points

%% R: measurement noise covariance matrix
% The associated block-daigonal covariance matrix of z
% R = diag(R1 ... Rn) = 0.1^2 * eye(length(z));

%% Starting index
i = 2;
j = 2;
nowTime = -0.01;

%% Initial estimate
x = [p_w(:,i); v_w(:,i-1)]; % easy as ground truth location

Ppos = diag([0.5 0.5 0.5]);
Pvel = diag([0.5 0.5 0.5]);
P = [Ppos zeros(3); zeros(3) Pvel];

%% Initialize storage matrices and figure
numCamMeasurements = size(observed_pts_c, 2);
numImuMeasurements = length(imuData);
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
distanceError = zeros(1, numPoses);
velocityError = zeros(1, numPoses);
process_params = cell(4,1);
obs_params = cell(5,1);


h = figure('Name','Position and Velocity Estimation', ...
           'NumberTitle','off','Position',[10 10 1000 600]);

ukf_N = length(x);

%% Begin Kalman filter
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
        
        process_params{1} = accel_i_measured(:,i); % usually known as u
        process_params{2} = dt;
        process_params{3} = q_w_i(:,i);
        process_params{4} = gravity;
        process_handle = @processModelPV;
        
        [x, P] = predictUKF(x, process_handle, process_params, ...
                            P, Q, ukf_alpha, ukf_beta);
        
        P
        i = i + 1;        
    else
        %% Correction Step
        
        % Perform correction step
        z = noisy_observed_pts_c(:,j);
        R = 0.1^2 * eye(length(z));
        
        q_world_IMU = q_w_i(:,j);
        p_IMU_camera = repmat(p_i_c, 1, 2*ukf_N+1);
        q_IMU_camera = repmat(q_i_c, 1, 2*ukf_N+1);
        p_world_pts = pts_w(1:3, :);
        
        obs_params{1} = q_world_IMU;
        obs_params{2} = p_IMU_camera;
        obs_params{3} = q_IMU_camera;
        obs_params{4} = p_world_pts;
        obs_params{5} = K;
        obs_handle = @measurementModelPV;
        
        [ x, P ] = correctUKF( x, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta );
        
        j = j + 1;
    end
    
    if (i < numImuMeasurements)
        
        %% Distance error
        distanceError(1,count) = norm(x(1:3) - p_w(:,i-1));
        velocityError(1,count) = norm(x(4:6) - v_w(:,i-1));
        
        %% Plot
        accumPoses(:,count) = x(1:3);
        count = count + 1;

        if mod(count, 10) == 1
            figure(h);
            clf

            subplot(2,2,[1, 3]);
            plot3(accumPoses(1,1:count-1), accumPoses(2,1:count-1), accumPoses(3,1:count-1),'-');
            hold on;
            plot3(p_w(1,1:i), p_w(2,1:i), p_w(3,1:i), 'g');
    %         hold on;
    %         plot3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r.');
            axis equal
            axis vis3d
            xlabel('x'); ylabel('y'); zlabel('z');
            grid on;
            title('Motion Estimation');

            subplot(2,2,2);
            plot(1:count,distanceError(1:count));
            maxErr = max(distanceError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            ylabel('Squared Error');
            title('Distance Error');

            subplot(2,2,4);
            plot(1:count,velocityError(1:count));
            maxErr = max(velocityError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            ylabel('Squared Error');
            title('Velocity Error');

            %pause
        end
    
    end
    
end

