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
timeStep = 0.05;                            % delta T
t = 0:timeStep:20;                          % simulation run time and time step

a_w_c = repmat([0.3 0.8 -0.1]', 1, length(t));   % constant linear acceleration for the camera
p0_w_c = [0 0 0]';                          % initial camera position in the world
v0_w_c = [0.3 0.8 -0.1]*timeStep';          % initial camera velocity

q_i_c = [ 0.7071 0 0 0.7071 ]';                       % rotation from IMU to camera
p_i_c = [ 10 0 0]';                          % translation from IMU to camera

numPoints = 100;                            % number of landmarks
pts_min = -50;                          
pts_max = 50;
pts_center = [300 100 0]';                  % mean landmark

gravity = [0 0 9.81]';                      % gravity


%% Derived parameters
nSteps = length(t);

camera_x = repmat([1 0 0]',1,nSteps);
camera_y = repmat([0 -1 0]',1,nSteps);
camera_z = repmat([0 0 -1]',1,nSteps);


%% Generate landmarks
pts_w = bsxfun(@plus, pts_min+(pts_max-pts_min).*rand(3,numPoints), pts_center);


%% Generate camera path first and find its orientation
q_w_c = zeros(4,nSteps);
v_w_c = zeros(3,nSteps);
p_w_c = zeros(3,nSteps);

v_w_c(:,1) = v0_w_c;
p_w_c(:,1) = p0_w_c;
q_w_c(:,1) = cameraOrientation(p_w_c(:,1), v_w_c(:,1), pts_center);

for i = 2:nSteps
    dt = t(i) - t(i-1);                 
    v_w_c(:,i) = v_w_c(:,i-1) + a_w_c(:,i-1)*dt;
    p_w_c(:,i) = p_w_c(:,i-1) + v_w_c(:,i-1)*dt + 0.5*a_w_c(:,i-1)*dt^2;
    
    q_w_c(:,i) = cameraOrientation(p_w_c(:,i), v_w_c(:,i), pts_center);
end


%% Position and orientation of IMU in the world frame 
q_w_i=zeros(4,nSteps);
p_w_i=zeros(3,nSteps);

for i=1:nSteps
    q_w_i(:,i)=rotation2quaternion(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)));
    p_w_i(:,i)=p_w_c(:,i)+(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)))*(-p_i_c);
end

v_w_i=bsxfun(@rdivide,diff(p_w_i,1,2),diff(t,1));
a_w_i=bsxfun(@rdivide,diff(v_w_i,1,2),diff(t(1:end-1),1));


%% Position camera axis throughout simulation
for i = 1:nSteps
   camera_x(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_x(:,i))*30; 
   camera_y(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_y(:,i))*30; 
   camera_z(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_z(:,i))*30; 
end


%% Plot
if plotFlag
    for i = 1:length(t) - 1
    
        figure(1);
        grid on;

        % plot points
        scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r', '.');
        hold on;
        scatter3(pts_center(1), pts_center(2), pts_center(3), 'b', '.');
        
        plot3([p_w_c(1,i) pts_center(1)], [p_w_c(2,i) pts_center(2)], [p_w_c(3,i) pts_center(3)],'m-'); 
        
        % plot camera path
        plot3(p_w_c(1,:), p_w_c(2,:), p_w_c(3,:), 'k-');
        
        % draw camera axis
        plot3([p_w_c(1,i) camera_x(1,i)], [p_w_c(2,i) camera_x(2,i)], [p_w_c(3,i) camera_x(3,i)], 'r');
        plot3([p_w_c(1,i) camera_y(1,i)], [p_w_c(2,i) camera_y(2,i)], [p_w_c(3,i) camera_y(3,i)], 'g');
        plot3([p_w_c(1,i) camera_z(1,i)], [p_w_c(2,i) camera_z(2,i)], [p_w_c(3,i) camera_z(3,i)], 'b');

        % draw imu
        plot3([p_w_c(1,i) p_w_i(1,i)], [p_w_c(2,i) p_w_i(2,i)], [p_w_c(3,i) p_w_i(3,i)], 'c-');
        
        axis equal; axis vis3d;    
        axis([0 350 0 150 -40 40]);
        xlabel('x'); ylabel('y'); zlabel('z');
        title(sprintf('frame %d/%d', i, length(t)-1));
        view([93 48]);
        
        hold off;
        
        pause(0.1);        
        
    end
    
end

