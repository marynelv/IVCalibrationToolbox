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

a_c_w = repmat([0.3 0.8 -0.1]', 1, length(t));   % constant linear acceleration for the camera
p0_c_w = [0 0 0]';                          % initial camera position in the world

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
q_c_w = zeros(4,nSteps);
v_c_w = zeros(3,nSteps);
p_c_w = zeros(3,nSteps);

p_c_w(:,1) = p0_c_w;
%q_c_w = 

for i = 2:nSteps
    dt = t(i) - t(i-1);                 
    v_c_w(:,i) = v_c_w(:,i-1) + a_c_w(:,i-1)*dt;
    p_c_w(:,i) = p_c_w(:,i-1) + v_c_w(:,i-1)*dt + 0.5*a_c_w(:,i-1)*dt^2;
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
    plot3(p_c_w(1,:), p_c_w(2,:), p_c_w(3,:), 'b-');
    
    axis equal; axis vis3d;
    
end
