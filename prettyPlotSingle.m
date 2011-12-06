function [p_w_c_estim,q_w_c_estim]=prettyPlotSingle(p_w_c, q_w_c, p_w_i, p_w_c_estim, q_w_c_estim, p_w_i_estim, pts_w, pts_center, i)

nSteps=size(p_w_i,2);
%p_w_c_estim=zeros(size(p_w_c));
%q_w_c_estim=zeros(size(q_w_c));

%i=1;

%for i=1:nSteps-10;
    %q_w_i(:,i)=rotation2quaternion(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)));
    %p_w_i(:,i)=p_w_c(:,i)+(quaternion2rotation(q_w_c(:,i))/(quaternion2rotation(q_i_c)))*(-p_i_c);    
    %q_w_c_estim(:,i)=rotation2quaternion(quaternion2rotation(q_w_i_estim(:,i))*quaternion2rotation(q_i_c_estim(:,i)));
    %p_w_c_estim(:,i)=p_w_i_estim(:,i)+quaternion2rotation(q_w_i_estim(:,i))*p_i_c_estim(:,i);
%end

camera_x = repmat([1 0 0]',1,nSteps);
camera_y = repmat([0 -1 0]',1,nSteps);
camera_z = repmat([0 0 -1]',1,nSteps);

%for i = 1:nSteps-10
   camera_x(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_x(:,i))*2; 
   camera_y(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_y(:,i))*2; 
   camera_z(:,i) = p_w_c(:,i) + quaternionRotate(q_w_c(:,i)', camera_z(:,i))*2; 
%end


camera_x_estim = repmat([1 0 0]',1,nSteps);
camera_y_estim = repmat([0 -1 0]',1,nSteps);
camera_z_estim = repmat([0 0 -1]',1,nSteps);

%for i = 1:nSteps-10
   camera_x_estim(:,i) = p_w_c_estim(:,i) + quaternionRotate(q_w_c_estim(:,i)', camera_x_estim(:,i))*2; 
   camera_y_estim(:,i) = p_w_c_estim(:,i) + quaternionRotate(q_w_c_estim(:,i)', camera_y_estim(:,i))*2; 
   camera_z_estim(:,i) = p_w_c_estim(:,i) + quaternionRotate(q_w_c_estim(:,i)', camera_z_estim(:,i))*2; 
%end


    
    %figure(1);
    grid on;
    
    % plot points
    thescattered=plot3(pts_w(1, :), pts_w(2, :), pts_w(3, :),'.');
    set(thescattered,'Color',[.2,.5,0]);
    hold on;
    %scatter3(pts_center(1), pts_center(2), pts_center(3), 'b', '.');
    
    
    %         plot3([p_w_c(1,i) pts_center(1)], [p_w_c(2,i) pts_center(2)], [p_w_c(3,i) pts_center(3)],'m-');
    
    % plot camera path
    plot3(p_w_c(1,1:i), p_w_c(2,1:i), p_w_c(3,1:i), 'w-', 'LineWidth',2);
    
    % draw camera axis
    plot3([p_w_c(1,i) camera_x(1,i)], [p_w_c(2,i) camera_x(2,i)], [p_w_c(3,i) camera_x(3,i)], 'r');
    plot3([p_w_c(1,i) camera_y(1,i)], [p_w_c(2,i) camera_y(2,i)], [p_w_c(3,i) camera_y(3,i)], 'r');
    plot3([p_w_c(1,i) camera_z(1,i)], [p_w_c(2,i) camera_z(2,i)], [p_w_c(3,i) camera_z(3,i)], 'r');
    
    % draw estimated camera path
    plot3(p_w_c_estim(1,1:i), p_w_c_estim(2,1:i), p_w_c_estim(3,1:i), 'm-','LineWidth',2);
    
    % draw estimated camera axis
    plot3([p_w_c_estim(1,i) camera_x_estim(1,i)], [p_w_c_estim(2,i) camera_x_estim(2,i)], [p_w_c_estim(3,i) camera_x_estim(3,i)], 'b');
    plot3([p_w_c_estim(1,i) camera_y_estim(1,i)], [p_w_c_estim(2,i) camera_y_estim(2,i)], [p_w_c_estim(3,i) camera_y_estim(3,i)], 'b');
    plot3([p_w_c_estim(1,i) camera_z_estim(1,i)], [p_w_c_estim(2,i) camera_z_estim(2,i)], [p_w_c_estim(3,i) camera_z_estim(3,i)], 'b');
    
    
    % draw imu
    plot3([p_w_c(1,i) p_w_i(1,i)], [p_w_c(2,i) p_w_i(2,i)], [p_w_c(3,i) p_w_i(3,i)], 'c-');
    
    % draw estimated imu
    plot3([p_w_c_estim(1,i) p_w_i_estim(1,i)], [p_w_c_estim(2,i) p_w_i_estim(2,i)], [p_w_c_estim(3,i) p_w_i_estim(3,i)], 'g-');
    
    axis equal; axis vis3d; camproj('p');
%     maxV = max([pts_w(1:3,:) p_w_c], [], 2);
%     minV = min([pts_w(1:3,:) p_w_c], [], 2);
%     axisLims = [minV maxV]'
%     
%     axis(axisLims(:)');
axis([-13, 5, -5, 15, -3, 6]);
    set(gca,'Color','k');
    
    
    xlabel('x'); ylabel('y'); zlabel('z');
    set(gcf,'Color',[.4,.4,.4]);
    %F=getframe(gcf);
    %imwrite(F.cdata,sprintf('plot/%03d.png',i));
    %title(sprintf('frame %d/%d', i, length(t)-1));
    view([-16 18]); %view([-41 36]);
    hold off;    
    %pause(0.01);
    
