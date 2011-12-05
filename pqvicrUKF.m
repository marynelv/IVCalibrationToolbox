% Position, orientation and velocity UKF
%clc
close all

%% UKF parameters
ukf_alpha = .01;
ukf_beta = 2;

%% x: state vector
% Composed by IMU position, rotation quaternion ([w x y z]') and velocity 
% in the world frame
% [p_w_i; q_w_i; v_w_i] = x(1:10); 
%% P: state covariance matrix 9-by-9
% NOTE: P is not 10-by-10 because we consider 3 deg of freedom for q_w_i

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
nowTime = imuData(i-1,3);

%% Initial estimate
clear x;
expected_rad_error = 1 * pi / 180;
init_rad_error = 0.2* expected_rad_error;
err_quat = matrix2quaternion(rotx(init_rad_error)*roty(init_rad_error)*rotz(init_rad_error));


x = [p_w(:,i); q_w_i(:,i); v_w(:,i-1); p_i_c+.1*randn(3,1); quaternionproduct(err_quat,q_i_c)]; % easy as ground truth location

%x = [p_w(:,i); q_w_i(:,i); v_w(:,i-1); p_i_c+.1*randn(3,1); q_i_c]; % easy as ground truth location
xstart=x;

Ppos = diag([.5 .5 .5]);
Pori = (10 * pi / 180)* eye(3);
Pvel = diag([0.5 0.5 0.5]);
Ppic=diag([.5 .5 .5]);
Pqic=(20*pi/180)*eye(3);
%P = [Ppos zeros(3, 6); zeros(3) Pori zeros(3); zeros(3,6) Pvel];
P=blkdiag(Ppos,Pori,Pvel,Ppic,Pqic);
Pstart=P;

%% Initialize storage matrices and figure
numCamMeasurements = size(observed_pts_c, 2);
numImuMeasurements = length(imuData);
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
accumQuat = NaN * ones(4,numPoses);
distanceError = zeros(1, numPoses);
picError=zeros(1,numPoses);
qicError=zeros(1,numPoses);
velocityError = zeros(1, numPoses);
orientationError = zeros(1, numPoses);
process_params = cell(4,1);
obs_params = cell(5,1);


h = figure('Name','Position, Orientation and Velocity Estimation', ...
           'NumberTitle','off','Position',[10 10 1000 600]);


%% Begin Kalman filter
count = 1;
while (i <= numImuMeasurements && j <= numCamMeasurements )
    
    % Read the timestamp for the next data input
    imuTime = imuData(i,3);
    camTime = camData(j,3);
    
    % Get previous orientation belief
    prev_q = x(4:7);
    prev_qic_q=x(14:17);
    
    if (imuTime <= camTime)
        %% Prediction step
        pastTime = nowTime;
        nowTime = imuTime;
        dt = nowTime - pastTime;
        
        u = [accel_i_measured(:,i); gyro_i_measured(:,i)];
        
        process_params{1} = u;
        process_params{2} = dt;
        process_params{3} = prev_q;
        process_params{4} = gravity;
        process_handle = @processModelPQVICr;
        
        %x_se = [x(1:3); 0; 0; 0; x(8:10)]; % State error vector with q in MRP
        x_se = [x(1:3); 0; 0; 0; x(8:10);x(11:13); 0;0;0];
        %[x_se, Ppred] = predictUKF(x_se, process_handle, process_params, ...
         %                      P(1:end-3,1:end-3), Q, ukf_alpha, ukf_beta);
        [x_se, Ppred] = predictUKF(x_se, process_handle, process_params, ...
                               P, Q, ukf_alpha, ukf_beta);
        
        mrp_error = x_se(4:6);
        % Convert MRP update vector to quaternion update
        norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
        dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
        
        q_error = [ dq0;
            bsxfun(@times,(1+dq0),mrp_error)];
        
        quat_new = quaternionproduct(q_error, prev_q);
        quat_new = quat_new./norm(quat_new);
       
        
        mrp_qic_error=x_se(13:15);
        norm_mrp_qic_error=sqrt(sum(mrp_qic_error.^2,1));
        dq_qic0=(1-norm_mrp_qic_error)./(1+norm_mrp_qic_error);
        q_qic_error=[dq_qic0; bsxfun(@times,1+dq_qic0,mrp_qic_error)];
        quat_qic_new=quaternionproduct(q_qic_error, prev_qic_q);
        quat_qic_new=quat_qic_new/norm(quat_qic_new);
        
        
        x'
        %x = [x_se(1:3); quat_new; x_se(7:9);x(end-2:end)];
        x = [x_se(1:3); quat_new; x_se(7:9);x_se(10:12);quat_qic_new];
        x'
        
        %P=blkdiag(Ppred,P(end-2:end,end-2:end));
        P=Ppred;
        
        i = i + 1;        
    else
        %% Correction Step
        
        % Perform correction step
        z = noisy_observed_pts_c(:,j);
        R = 0.1^2 * eye(length(z));
        
        x_se = [x(1:3); 0; 0; 0; x(8:10);x(11:13);0;0;0]; % State error vector with q in MRP
        ukf_N = length(x_se);
        
        %p_IMU_camera = repmat(p_i_c, 1, 2*ukf_N+1);
        %q_IMU_camera = repmat(q_i_c, 1, 2*ukf_N+1);
        p_world_pts = pts_w(1:3, :);
        
        obs_params{1} = prev_q;
        obs_params{2} = prev_qic_q;
        obs_params{3} = p_world_pts;
        obs_params{4} = K;
        obs_handle = @measurementModelPQVICr;
        
        [ x_se, P ] = correctUKF( x_se, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta );
        
        mrp_error = x_se(4:6);
        % Convert MRP update vector to quaternion update
        norm_mrp_error = sqrt(sum(mrp_error.^2, 1));
        dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
        
        q_error = [ dq0;
            bsxfun(@times,(1+dq0),mrp_error)];
        
        quat_new = quaternionproduct(q_error, prev_q);
        quat_new = quat_new./norm(quat_new);
        
        
        mrp_qic_error=x_se(13:15);
        norm_mrp_qic_error=sqrt(sum(mrp_qic_error.^2,1));
        dq_qic0=(1-norm_mrp_qic_error)./(1+norm_mrp_qic_error);
        q_qic_error=[dq_qic0; bsxfun(@times,1+dq_qic0,mrp_qic_error)];
        quat_qic_new=quaternionproduct(q_qic_error, prev_qic_q);
        quat_qic_new=quat_qic_new/norm(quat_qic_new);
        
        
        x = [x_se(1:3); quat_new; x_se(7:9);x_se(10:12);quat_qic_new];
        
         j = j + 1;
    end
    
    if (i < numImuMeasurements)
        
        %% Distance error
        distanceError(1,count) = norm(x(1:3) - p_w(:,i-1));
        velocityError(1,count) = norm(x(8:10) - v_w(:,i-1));
        orientationError(1,count) = findQuaternionError(x(4:7), q_w_i(:,i-1));
        picError(1,count)=norm(x(11:13)-p_i_c);
        qicError(1,count)=findQuaternionError(x(14:17),q_i_c);

        %% Plot
        accumPoses(:,count) = x(1:3);
        count = count + 1;

        if mod(count, 10) == 1
            figure(h);
            clf

            subplot(5,2,[1, 3, 5, 7, 9]);
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

            subplot(5,2,2);
            plot(1:count,distanceError(1:count));
            maxErr = max(distanceError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            ylabel('Squared Error');
            title('Distance Error');

            subplot(5,2,4);
            plot(1:count,velocityError(1:count));
            maxErr = max(velocityError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            ylabel('Squared Error');
            title('Velocity Error');

            subplot(5,2,6);
            plot(1:count,orientationError(1:count));
            maxErr = max(orientationError);
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            ylabel('Squared Error');
            title('Orientation Error');
            
            subplot(5,2,8);
            plot(1:count,picError(1:count));
            maxErr = max(picError)+.1;
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            ylabel('Squared Error');
            title('IMU-Camera Translation Error');

            subplot(5,2,10);
            plot(1:count,qicError(1:count));
            maxErr = max(qicError)+.1;
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            ylabel('Squared Error');
            title('IMU-Camera Rotation Error');
            %pause
        end
    
    end
    
end

