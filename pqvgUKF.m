% Position, orientation, velocity and gravity UKF
%clc
close all
clear x;

%% UKF parameters
ukf_alpha = 0.1;
ukf_beta = 2;

%% x: state vector
% Composed by IMU position, rotation quaternion ([w x y z]'), velocity 
% and gravity in the world frame
% [p_w_i; q_w_i; v_w_i, g_w] = x(1:13); 
%% P: state covariance matrix 9-by-9
% NOTE: P is not 13-by-13 because we consider 3 deg of freedom for q_w_i

%% u: process inputs
% Composed by measured IMU acceleration and rotational velocity
% u = [accel_i_measured(1:3, i); gyro_i_measured(1:3, i)] 

%% n: process noise

%% Q: process noise covariance matrix
Qacc = eye(3) * 0.1^2;
Qrot = eye(3) * 0.001 * 300 * pi/180; % rad/s
Q = [Qacc zeros(3); zeros(3) Qrot];

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
nowTime = imuData(i,3);

%% Initial estimate
iniPos = p_w(:,i) + 1*rand(3,1);   % initial position in world frame
iniQ   = q_w_i(:,i) + 0.02*rand(4,1);  % initial orientation in world frame
iniQ = iniQ ./ norm(iniQ); 
iniV   = v_w(:,i) + .4*rand(3,1);   % initial velocity in world frame
C_q_w_i_0 = quaternion2matrix(iniQ);
iniG   = -gravity + 0.1*rand(3,1); % initial gravity in world frame
x(1:13,1) = [iniPos; iniQ; iniV; iniG]; 

Ppos = eye(3)*0.5;
Pori = (10 * pi / 180)* eye(3);
Pvel = diag([0.5 0.5 0.5]);
Pgra = eye(3)*1e-8; % just very small, though ideally it would be zero
P = [Ppos zeros(3, 9); ...
     zeros(3) Pori zeros(3,6); ...
     zeros(3,6) Pvel zeros(3); ...
     zeros(3,9) Pgra];

%% Initialize storage matrices and figure
numCamMeasurements = size(observed_pts_c, 2);
numImuMeasurements = length(imuData);
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
accumQuat = NaN * ones(4,numPoses);
distanceError = zeros(1, numPoses);
velocityError = zeros(1, numPoses);
orientationError = zeros(1, numPoses);
gravityError = zeros(1, numPoses);
process_params = cell(3,1);
obs_params = cell(5,1);


h = figure('Name','Position, Orientation, Velocity and Gravity Estimation', ...
           'NumberTitle','off','Position',[10 10 1000 600]);

%% Begin Kalman filter
count = 1;
while (i <= numImuMeasurements - 3 && j <= numCamMeasurements - 3 )
    
    % Read the timestamp for the next data input
    imuTime = imuData(i,3);
    camTime = camData(j,3);
    
    % Get previous orientation belief
    prev_q = x(4:7);
    x_se = [x(1:3); 0; 0; 0; x(8:13)]; % State error vector with q in MRP
    
%    if (imuTime <= camTime)
    if (imuTime <= camTime)
        %% Prediction step
        pastTime = nowTime;
        nowTime = imuTime;
        dt = nowTime - pastTime;
        
        u = [accel_i_measured(:,i); gyro_i_measured(:, i)];
        
        process_params{1} = u;
        process_params{2} = dt;
        process_params{3} = prev_q;
        process_handle = @processModelPQVG;
        
        [x_se, P] = predictUKF(x_se, process_handle, process_params, ...
                               P, Q, ukf_alpha, ukf_beta);
        
        mrp_error = x_se(4:6);
        % Convert MRP update vector to quaternion update
        norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
        dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
        
        q_error = [ dq0;
            bsxfun(@times,(1+dq0),mrp_error)];
        
        quat_new = quaternionproduct(q_error, prev_q);
        quat_new = quat_new./norm(quat_new);
        
        x = [x_se(1:3); quat_new; x_se(7:12)];
        
        %P
        i = i + 1;        
    else
        %% Correction Step
        
        % Perform correction step
        z = noisy_observed_pts_c(:,j);
        R = 0.1^2 * eye(length(z));
        
        ukf_N = length(x_se);
        
        p_IMU_camera = repmat(p_i_c, 1, 2*ukf_N+1);
        q_IMU_camera = repmat(q_i_c, 1, 2*ukf_N+1);
        p_world_pts = pts_w(1:3, :);
        
        obs_params{1} = prev_q;
        obs_params{2} = p_IMU_camera;
        obs_params{3} = q_IMU_camera;
        obs_params{4} = p_world_pts;
        obs_params{5} = K;
        obs_handle = @measurementModelPQVG;
        
        [ x_se, P ] = correctUKF( x_se, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta );
        
        mrp_error = x_se(4:6);
        % Convert MRP update vector to quaternion update
        norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
        dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
        
        q_error = [ dq0;
            bsxfun(@times,(1+dq0),mrp_error)];
        
        quat_new = quaternionproduct(q_error, prev_q);
        quat_new = quat_new./norm(quat_new);
        
        x = [x_se(1:3); quat_new; x_se(7:12)];
        
        j = j + 1;
    end
    
    x(11:13)
    
    if (i < numImuMeasurements)
        
        %% Distance error
        distanceError(1,count) = norm(x(1:3) - p_w(:,i-1));
        orientationError(1,count) = findQuaternionError(x(4:7), q_w_i(:,i-1));
        velocityError(1,count) = norm(x(8:10) - v_w(:,i-1));
        gravityError(1,count) = norm(x(11:13) - (-gravity));
        
        %% Plot
        accumPoses(:,count) = x(1:3);

        if mod(count, 10) == 1
            figure(h);
            clf

            subplot(3,2,[1, 2]);
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

            subplot(3,2,3);
            plot(1:count,distanceError(1:count));
            maxErr = max(distanceError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            ylabel('Squared Error');
            title('Distance Error');

            subplot(3,2,5);
            plot(1:count,velocityError(1:count));
            maxErr = max(velocityError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            ylabel('Squared Error');
            title('Velocity Error');

            subplot(3,2,4);
            plot(1:count,orientationError(1:count));
            maxErr = max(orientationError);
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            ylabel('Squared Error');
            title('Orientation Error');
            
            subplot(3,2,6);
            plot(1:count,gravityError(1:count));
            maxErr = max(gravityError);
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            ylabel('Squared Error');
            title('Gravity Error');

            %pause
        end

%     if (distanceError(count) > 0.05)
%         display(sprintf('Distance error is > 0.05 at i=%d, j=%d at count = %d', i, j, count));
%        pause; 
%     end
    
        count = count + 1;
    
    end
    
end

