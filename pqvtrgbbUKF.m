% Position, orientation, velocity, I-C translation, I-C rotation, gravity 
% and biases UKF
%clc
close all

if (~exist('plotFlag')),
    plotFlag = 1; % make 1 to plot
end
saveFlag = 0; % make 1 to save plot

%% UKF parameters
ukf_alpha = .01;
ukf_beta = 2;

%% x: state vector
% Composed by IMU position, rotation quaternion ([w x y z]') and velocity 
% in the world frame
% [p_w_i; q_w_i; v_w_i; q_i_c; q_i_c; gravity; bias_accel; bias_gyro] = x(1:26); 
%% P: state covariance matrix 24-by-26
% NOTE: P is not 26-by-26 because we consider 3 deg of freedom for the
% rotations

%% u: process inputs
% Composed by measured IMU acceleration and rotational velocity
% u = [accel_i_measured(1:3, i); gyro_i_measured(1:3, i)] 

%% n: process noise

%% Q: 12x12 process noise covariance matrix 
Qgw = eye(3) * 0.0196^2;   % noise gyro bias (more or less as in the data sheet)
Qaw = eye(3) * 0.0001^2;    % noise accel bias (more or less as synthesized)
Qacc = eye(3) * 0.1^2;  
Qrot = eye(3) * 0.001 * 300 * pi/180; % rad/s
Q = blkdiag(Qacc, Qrot, Qaw, Qgw);

%% z: measurements
% See section 4.3 Measurement Model on page 11
% z is a 2n-by-1 column vector of observed pixel coordinates in the
% form [x1 y1 ... xn yn]' where n is the number of 3D feature points

%% R: measurement noise covariance matrix
% The associated block-daigonal covariance matrix of z
% R = diag(R1 ... Rn) = 0.1^2 * eye(length(z));

%% Starting index
%i = lengthLowPass+1;
%j = lengthLowPass+1;
%nowTime = imuData(i-1,3);
i = 2;
j = 2;
nowTime = imuData(i-1,3);


%% Initial estimate
clear x;
mrpStd = @(deg) tan(deg*pi/720); % mrp error = tan((deg*pi)/(4*180))

iniPos = p_w(:,i) + rand(3,1).*(0.08^2);   % initial position in world frame
iniV   = v_w(:,i-1) + rand(3,1).*(0.3^2);  % initial velocity in world frame
iniQMRPerror = rand(3,1).*(mrpStd(8)^2);   % 8 deg of error for initial orientation 
norm_mrp_error = sqrt(sum(iniQMRPerror.^2, 1));
dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
iniQerror = [ dq0; bsxfun(@times,(1+dq0),iniQMRPerror)];
iniQ   = quaternionproduct(iniQerror,q_w_i(:,i));  % initial orientation in world frame
iniQ = iniQ ./ norm(iniQ); 

iniP_i_c = p_i_c+.1*randn(3,1);      
iniQicMRPerror = rand(3,1).*(mrpStd(5)^2);   % 5 deg of error for initial imu-cam orientation 
norm_mrp_error = sqrt(sum(iniQicMRPerror.^2, 1));
dq0 = (1 - norm_mrp_error) ./ (1 + norm_mrp_error);
iniQicerror = [ dq0; bsxfun(@times,(1+dq0),iniQicMRPerror)];
iniQ_i_c   = quaternionproduct(iniQicerror,q_i_c);  % initial orientation in world frame
iniQ_i_c = iniQ_i_c ./ norm(iniQ_i_c);

iniG=-gravity+.1*randn(3,1);

iniBa = bias_accel(:,i) + + 0.1*rand(3,1);
iniBg = bias_gyro(:,i) + + 0.1*rand(3,1);

x = [iniPos; iniQ; iniV; iniP_i_c; iniQ_i_c; iniG; iniBa; iniBg]; 

Ppos = eye(3).*(.1^2);
Pori = eye(3).*(mrpStd(10)^2);
Pvel = eye(3).*(.5^2);
Ppic=eye(3).*1e-4;
Pqic=eye(3).*(mrpStd(7)^2);
Pgra = diag([1.7 1.7 0.15].^2); % just very small, though ideally it would be zero
Pba = eye(3)*0.001^2;
Pbg = eye(3)*0.02^2;
P=blkdiag(Ppos,Pori,Pvel,Ppic,Pqic,Pgra,Pba,Pbg);

%% Initialize storage matrices and figure
numCamMeasurements = size(observed_pts_c, 2) - j - 1;
numImuMeasurements = length(imuData) - i - 1;
numPoses = numImuMeasurements + numCamMeasurements;
accumPoses = zeros(3,numPoses);
accumQuat = NaN * ones(4,numPoses);
distanceError = zeros(1, numPoses);
orientationError = zeros(1, numPoses);
velocityError = zeros(1, numPoses);
picError=zeros(1,numPoses);
qicError=zeros(1,numPoses);
gravityError = zeros(1, numPoses);
biasAccelError = zeros(1,numPoses);
biasGyroError = zeros(1,numPoses);
process_params = cell(3,1);
obs_params = cell(5,1);


accum_pwc = zeros(3,numPoses);
accum_pwi = zeros(3,numPoses);
accum_pwi_estim = zeros(3,numPoses);
accum_qwc = zeros(4,numPoses);
accum_qwi = zeros(4,numPoses);
accum_qwi_estim = zeros(4,numPoses);
accum_pic_estim = zeros(3,numPoses);
accum_qic_estim = zeros(4,numPoses);
accum_pwc_estim = zeros(3,numPoses);
accum_qwc_estim = zeros(4,numPoses);
accum_biasaccel_estim = zeros(3,numPoses);
accum_biasgyro_estim = zeros(3,numPoses);

if plotFlag == 1,
    h = figure('Name','Position, Orientation and Velocity Estimation', ...
               'NumberTitle','off','Position',[10 10 1000 600]);
    h1 = figure('Name', 'Gyro Bias', ...
                'NumberTitle', 'off', 'Position',[50 50 500 300]);
    h2 = figure('Name', 'Accel Bias', ...
                'NumberTitle', 'off', 'Position',[50 50 500 300]);
end

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
        process_handle = @processModelPQVTRGBB;
        
        % State error vector with q and q_i_c as MRP
        x_se = [x(1:3); 0; 0; 0; x(8:13); 0; 0; 0; x(18:26)];
        [x_se, Ppred] = predictUKF(x_se, process_handle, process_params, ...
                               P, Q, ukf_alpha, ukf_beta);
        
        % Convert MRP update vectors to quaternion update
        mrp_error = x_se(4:6);
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
        
        x = [x_se(1:3); quat_new; x_se(7:12); quat_qic_new; x_se(16:24)];

        P=Ppred;
        
        i = i + 1;        
    else
        %% Correction Step
        
        % Perform correction step
        z = noisy_observed_pts_c(:,j);
        R = 0.1^2 * eye(length(z));
        
        % State error vector with q and q_i_c as MRP
        x_se = [x(1:3); 0; 0; 0; x(8:13); 0; 0; 0; x(18:26)]; % State error vector with q in MRP
        
        obs_params{1} = prev_q;
        obs_params{2} = prev_qic_q;
        obs_params{3} = pts_w(1:3, :); %landmarks in the world;
        obs_params{4} = K;
        obs_handle = @measurementModelPQVTRGBB;
        
        [ x_se, P ] = correctUKF( x_se, P, R, z, obs_handle, obs_params, ukf_alpha, ukf_beta );
        
        % Convert MRP update vectors to quaternion update
        mrp_error = x_se(4:6);
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
        
        
        x = [x_se(1:3); quat_new; x_se(7:12); quat_qic_new; x_se(16:24)];
        
        j = j + 1;
    end
    
    if (i < numImuMeasurements)
        
        
        %% Distance error
        distanceError(1,count) = norm(x(1:3) - p_w(:,i-1));
        velocityError(1,count) = norm(x(8:10) - v_w(:,i-1));
        orientationError(1,count) = findQuaternionError(x(4:7), q_w_i(:,i-1));
        picError(1,count)=norm(x(11:13)-p_i_c);
        qicError(1,count)=findQuaternionError(x(14:17),q_i_c);
        gravityError(1,count) = norm(x(18:20) - (-gravity));
        biasAccelError(1,count) = norm(x(21:23) - bias_accel(:,i-1));
        biasGyroError(1,count) = norm(x(24:26) - bias_gyro(:,i-1));
        
        %% Plot
        accumPoses(:,count) = x(1:3);
        accum_pwi(:,count)=p_w(:,i-1);
        accum_pwi_estim(:,count)=x(1:3);
        accum_pwc(:,count)=p_w_c(:,i-1);
        accum_qwi(:,count)=q_w_i(:,i-1);
        accum_qwc(:,count)=q_w_c(:,i-1);
        accum_qwi_estim(:,count)=x(4:7);
        accum_pic_estim(:,count)=x(11:13);
        accum_qic_estim(:,count)=x(14:17);
        
        accum_qwc_estim(:,count)=rotation2quaternion(quaternion2rotation(accum_qwi_estim(:,count))*quaternion2rotation(accum_qic_estim(:,count)));
        accum_pwc_estim(:,count)=accum_pwi_estim(:,count)+quaternion2rotation(accum_qwi_estim(:,count))*accum_pic_estim(:,count);        
        
        accum_biasaccel_estim(:,count) = x(21:23);
        accum_biasgyro_estim(:,count) = x(24:26);
        
        count = count + 1;

        if mod(count, 10) == 1 && plotFlag == 1
            figure(h);
            clf

            subplot(4,3,[1, 4, 7, 10]);
            %plot3(accumPoses(1,1:count-1), accumPoses(2,1:count-1), accumPoses(3,1:count-1),'-');
            %hold on;
            %plot3(p_w(1,1:i), p_w(2,1:i), p_w(3,1:i), 'g');
    %         hold on;
    %         plot3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r.');
            %axis equal
            %axis vis3d
            %xlabel('x'); ylabel('y'); zlabel('z');
            %grid on;
            [accum_pwc_estim,accum_qwc_estim]=prettyPlotSingle(...
                accum_pwc,accum_qwc,accum_pwi,...
                accum_pwc_estim,accum_qwc_estim,accum_pwi_estim,...
                pts_w,pts_center,count-1);        
            
            title(sprintf('Motion Estimation (Frame %d)', count));

            subplot(4,3,2);
            plot(1:count,distanceError(1:count));
            maxErr = max(distanceError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            %ylabel('Squared Error');
            title('Distance Error');

            subplot(4,3,3);
            plot(1:count,velocityError(1:count));
            maxErr = max(velocityError);
            axis([0 numPoses 0 maxErr]);
    %        xlabel('Time');
            %ylabel('Squared Error');
            title('Velocity Error');

            subplot(4,3,5);
            plot(1:count,orientationError(1:count));
            maxErr = max(orientationError);
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
           % ylabel('Squared Error');
            title('Orientation Error');
            
            
            subplot(4,3,6);
            plot(1:count,gravityError(1:count));
            maxErr = max(gravityError);
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            %ylabel('Squared Error');
            title('Gravity Error');
            
            subplot(4,3,8);
            plot(1:count,picError(1:count));
            maxErr = max(picError)+.1;
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            %ylabel('Squared Error');
            title('IMU-Camera Translation Error');

            subplot(4,3,9);
            plot(1:count,qicError(1:count));
            maxErr = max(qicError)+.1;
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
           % ylabel('Squared Error');
            title('IMU-Camera Rotation Error');
            
            subplot(4,3,11);
            plot(1:count,biasAccelError(1:count));
            maxErr = max(biasAccelError)+.1;
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
           % ylabel('Squared Error');
            title('Accel Bias Error');

            subplot(4,3,12);
            plot(1:count,biasGyroError(1:count));
            maxErr = max(biasGyroError)+.1;
            axis([0 numPoses 0 maxErr]);
            xlabel('Time');
            %ylabel('Squared Error');
            title('Gyro Bias Error');
            if (saveFlag),
                F=getframe(gcf);
                imwrite(F.cdata,sprintf('plot/%03d.png',i));
            end
            %subplot(4,3,[1, 4, 7, 10]);
            %F=getframe(gca);
            %imwrite(F.cdata,sprintf('motion/%03d.png',i));
            
            %pause
            %saveas(gcf,sprintf('plot/%03d.png',i));
            subplot(4,3,[1, 4, 7, 10]);
            if (saveFlag),
                saveas(gca,sprintf('motion/%03d.png',i));
            end
            %print('-dpng','-r300','-opengl',sprintf('plot/%03d.png',i));
            
            
            figure(h1);
            clf
            subplot(1,2,1);
            hold on;
            plot(1:i-1,bias_gyro(1,1:i-1),'r-');
            plot(1:i-1,bias_gyro(2,1:i-1),'g-');
            plot(1:i-1,bias_gyro(3,1:i-1),'b-');
            title('Real Gyro Bias');
            subplot(1,2,2);
            hold on;
            plot(1:count,accum_biasgyro_estim(1,1:count),'r-');
            plot(1:count,accum_biasgyro_estim(2,1:count),'g-');
            plot(1:count,accum_biasgyro_estim(3,1:count),'b-');
            title('Estimated Gyro Bias');
            
            figure(h2);
            clf
            subplot(1,2,1);
            hold on;
            plot(1:i-1,bias_accel(1,1:i-1),'r-');
            plot(1:i-1,bias_accel(2,1:i-1),'g-');
            plot(1:i-1,bias_accel(3,1:i-1),'b-');
            title('Real Accel Bias');
            subplot(1,2,2);
            hold on;
            plot(1:count,accum_biasaccel_estim(1,1:count),'r-');
            plot(1:count,accum_biasaccel_estim(2,1:count),'g-');
            plot(1:count,accum_biasaccel_estim(3,1:count),'b-');
            title('Estimated Accel Bias');
            
        end
    
    end
    
end

