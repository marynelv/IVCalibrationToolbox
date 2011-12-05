% IMU Simulation with camera directed towards mean landmark
%
% Coordinate frames:
% w: world frame
% i: IMU frame
% c: camera (right-down-forward) frame
%
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
%rng(1);                                     % repeatable simulation results


%% Setup parameters
plotFlag = 1;                               % want a plot?
timeStep = 0.01;                            % delta T
t = 0:timeStep:10;                          % simulation run time and time step

a_w_c = repmat([-0.3 0.8 -0.1]', 1, length(t));   % constant linear acceleration for the camera
p0_w_c = [0 0 0]';                          % initial camera position in the world
v0_w_c = [0.3 0.8 -0.1]*timeStep';          % initial camera velocity

q_i_c = [ 0.7071 0 0 0.7071 ]';                       % rotation from IMU to camera
% p_i_c = [ 10 0 0]';                          % translation from IMU to camera
%q_i_c = [ 1 0 0 0 ]';                       % rotation from IMU to camera
p_i_c = [ 10 0 0]';                          % translation from IMU to camera


numPoints = 100;                            % number of landmarks
pts_min = -5;                          
pts_max = 5;
pts_center = [10 10 0]';                    % mean landmark

std_pixel_noise = 0.1;
std_v_w = 0.1;

gravity = [0 0 9.81]';                      % gravity
%gravity = [0 0 0]';                      % gravity

image_width = 640;                          % image width
image_height = 480;                         % image height
f = 0.7;                                      % focal length

%% Camera stuff
pts_proj = zeros(2,numPoints);              % landmarks projected into the image plane
px = image_width/2; py = image_height/2;    % principal point
K = [f*image_width 0 px; 0 f*image_height py; 0 0 1];                % intrinsic parameters

%% Generate landmarks
pts_w = bsxfun(@plus, pts_min+(pts_max-pts_min).*rand(3,numPoints), pts_center);


%% Generate camera path first and find its orientation
nSteps = length(t);
q_w_c = zeros(4,nSteps);
v_w_c = zeros(3,nSteps);
gg=fspecial('gaussian',[1,6*2+1],1);
p_w_c1=generatePosBspline(pts_w,nSteps+size(gg,2)-1);

p_w_c=[];
for j=1:3
    if j==1
        p_w_c=conv(p_w_c1(j,:),gg,'valid');
    else
        p_w_c=[p_w_c;conv(p_w_c1(j,:),gg,'valid')];
    end
end
p0_w_c=p_w_c(:,1);
%p_w_c = zeros(3,nSteps);

%v_w_c(:,1) = v0_w_c;
%p_w_c(:,1) = p0_w_c;
%q_w_c(:,1) = cameraOrientation(p_w_c(:,1), v_w_c(:,1), pts_center);

for i = 1:nSteps-1
    %dt = t(i) - t(i-1);                 
    %v_w_c(:,i) = v_w_c(:,i-1) + a_w_c(:,i-1)*dt;
    %p_w_c(:,i) = p_w_c(:,i-1) + v_w_c(:,i-1)*dt + 0.5*a_w_c(:,i-1)*dt^2;
    
    q_w_c(:,i) = cameraOrientation(p_w_c(:,i), p_w_c(:,i+1)-p_w_c(:,i), pts_center);
end

q_w_c(:,end)=q_w_c(:,end-1);

camera_x = repmat([1 0 0]',1,nSteps);
camera_y = repmat([0 -1 0]',1,nSteps);
camera_z = repmat([0 0 -1]',1,nSteps);

p0_w_c=p_w_c(:,1);


%% Position and orientation of IMU in the world frame 
q_w_i=zeros(4,nSteps);
p_w_i=zeros(3,nSteps);

for i=1:nSteps
    q_w_i(:,i)=rotation2quaternion(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)));
    p_w_i(:,i)=p_w_c(:,i)+(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)))*(-p_i_c);
end

%v_w_i=bsxfun(@rdivide,diff(p_w_i,1,2),diff(t,1));
%a_w_i=bsxfun(@rdivide,diff(v_w_i,1,2),diff(t(1:end-1),1));
%v_w_i=[v_w_i,v_w_i(:,end)];
%a_w_i=[a_w_i,a_w_i(:,end-1:end)];

[v_i,a_i,omega_i,~,v_w]=getVelocityAcceleration(p_w_i,q_w_i,t,gravity);


%% Position camera axis throughout simulation
for i = 1:nSteps
   camera_x(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_x(:,i))*5; 
   camera_y(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_y(:,i))*5; 
   camera_z(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_z(:,i))*5; 
end


%% Projected points
observed_pts_c = NaN * ones(2*numPoints, length(t));
for i = 1:length(t)
        % [px py]' = K*R[ I | -C ] * [x y z 1]' 
        for p = 1:numPoints
%            xyz = K*quaternion2rotation(q_w_c(:,i))'*(pts_w(:,p) - p_w_c(:,i));
           xyz =  K*quaternion2rotation(q_w_c(:,i))'*[eye(3) -p_w_c(:,i)]*[pts_w(:,p); 1];
           pts_proj(1,p) = xyz(1)/xyz(3);
           pts_proj(2,p) = xyz(2)/xyz(3);
        end
        observed_pts_c(:,i)=pts_proj(:);
end

%% Rename stuff like before
a_w = a_i;
%v_w = v_i;
p_w = p_w_i;

std_dev_noise_accel = 0.1;
std_dev_bias_accel = 0;
std_dev_noise_gyro = 0.1;

bias_accel = zeros(size(a_i));
noise_accel = std_dev_noise_accel*randn(size(a_i));
accel_i_measured = a_i + bias_accel + noise_accel;

%w = repmat([0 0 0]', 1, length(t));
w=omega_i;
bias_gyro = zeros(size(w));
noise_gyro = std_dev_noise_gyro*randn(size(w));
gyro_i_measured = w + bias_gyro + noise_gyro;

imuData = zeros(length(t), 31);
imuData(:,3) = t;
imuData(:,17:19) = gyro_i_measured';
imuData(:,29:31) = accel_i_measured';

camData = zeros(length(t), 3);
camData(:,3) = t;


%% Create noisy measurements
noisy_v_w = v_w + std_v_w*randn(size(v_w));
noisy_observed_pts_c = observed_pts_c + std_pixel_noise*randn(size(observed_pts_c));

plotSynthesizeLookAtLandmarksWithRotations
% %% Plot
% if plotFlag
%     for i = 1:length(t) - 1
%         
%         figure(1);
%         grid on;
% 
%         % plot points
%         subplot(1,2,1);
%         scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r', '.');
%         hold on;
%         scatter3(pts_center(1), pts_center(2), pts_center(3), 'b', '.');
%         
% %         plot3([p_w_c(1,i) pts_center(1)], [p_w_c(2,i) pts_center(2)], [p_w_c(3,i) pts_center(3)],'m-'); 
%         
%         % plot camera path
%         plot3(p_w_c(1,:), p_w_c(2,:), p_w_c(3,:), 'k-');
%         
%         % draw camera axis
%         plot3([p_w_c(1,i) camera_x(1,i)], [p_w_c(2,i) camera_x(2,i)], [p_w_c(3,i) camera_x(3,i)], 'r');
%         plot3([p_w_c(1,i) camera_y(1,i)], [p_w_c(2,i) camera_y(2,i)], [p_w_c(3,i) camera_y(3,i)], 'g');
%         plot3([p_w_c(1,i) camera_z(1,i)], [p_w_c(2,i) camera_z(2,i)], [p_w_c(3,i) camera_z(3,i)], 'b');
% 
%         % draw imu
%         plot3([p_w_c(1,i) p_w_i(1,i)], [p_w_c(2,i) p_w_i(2,i)], [p_w_c(3,i) p_w_i(3,i)], 'c-');
%         
%         axis equal; axis vis3d;    
%         axis([-10 30 -5 45 -30 30]);
%         xlabel('x'); ylabel('y'); zlabel('z');
%         title(sprintf('frame %d/%d', i, length(t)-1));
%         view([-41 36]);
%         hold off;
%            
%         subplot(1,2,2);
%         scatter(observed_pts_c(1:2:end,i), observed_pts_c(2:2:end,i), 'r');
%         axis equal;
%         axis([0 image_width 0 image_height]);
%         xlabel('x'); ylabel('-y');
%         title('Camera image');
%         
%         pause(0.1);        
%         
%     end
%     
% end
% 
