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
rng(1);                                     % repeatable simulation results

%% Setup parameters
plotFlag = 1;                               % want a plot?
t = 0:0.01:20;                              % simulation run time and time step

a_w_c = repmat([0.3 0.8 -0.1]', 1, length(t));   % constant linear acceleration for the camera
p0_w_c = [0 0 0]';                          % initial camera position in the world

q_i_c = [ 1 0 0 0 ]';                       % rotation from IMU to camera
p_i_c = [ 0 0 0]';                          % translation from IMU to camera

numPoints = 100;                            % number of landmarks
pts_min = -50;                          
pts_max = 50;
pts_center = [300 100 0]';                  % mean landmark

gravity = [0 0 9.81]';                      % gravity

%% Derived parameters
nSteps = length(t);

%% Generate landmarks
pts_w = bsxfun(@plus, pts_min+(pts_max-pts_min).*rand(3,numPoints), pts_center);


%% Generate camera path first and find its orientation
q_w_c = zeros(4,nSteps);
v_w_c = zeros(3,nSteps);
p_w_c = zeros(3,nSteps);

p_w_c(:,1) = p0_w_c;

for i = 2:nSteps
    dt = t(i) - t(i-1);                 
    v_w_c(:,i) = v_w_c(:,i-1) + a_w_c(:,i-1)*dt;
    p_w_c(:,i) = p_w_c(:,i-1) + v_w_c(:,i-1)*dt + 0.5*a_w_c(:,i-1)*dt^2;
    
    q_w_c(:,i) = cameraOrientation(p_w_c(:,i), v_w_c(:,i)/norm(v_w_c(:,i)), pts_center);
end


%% Plot

if plotFlag
    figure(1);
    xlabel('x'); ylabel('y'); zlabel('z');
    grid on;
    hold on;
    
    % plot points
    scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r', '.');
    
    % plot camera path
    plot3(p_w_c(1,:), p_w_c(2,:), p_w_c(3,:), 'b-');
    
    axis equal; axis vis3d;
    
end

%% position and orientation of IMU in the world frame 
q_w_i=zeros(4,nSteps);
p_w_i=zeros(3,nSteps);

for i=2:nSteps
    q_w_i(:,i)=rotation2quaternion(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)));
    p_w_i(:,i)=p_w_c(:,i)+quaternion2rotation(q_w_c(:,i))*(-p_i_c);
end

v_w_i=diff(p_w_i,1,2)./diff(t,1);
a_w_i=diff(v_w_i,1,2)./diff(t(1:end-1),1);