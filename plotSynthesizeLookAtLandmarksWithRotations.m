%% Plot
if plotFlag

    f = figure('position',[20 20 1000 600]);
    
    grid on;

    for i = 1:10:length(t) 
        drawnow

        % plot points
        subplot('position',[0,0.15,0.45,0.7]);
        scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 10, [.2,.5,0], 'filled');
        hold on;
        scatter3(pts_center(1), pts_center(2), pts_center(3), 3, [.2,.5,0]);
        
        
%         plot3([p_w_c(1,i) pts_center(1)], [p_w_c(2,i) pts_center(2)], [p_w_c(3,i) pts_center(3)],'m-'); 
        
        % plot camera path
        plot3(p_w_c(1,:), p_w_c(2,:), p_w_c(3,:), 'm-');
        
        % draw camera axis
        plot3([p_w_c(1,i) camera_x(1,i)], [p_w_c(2,i) camera_x(2,i)], [p_w_c(3,i) camera_x(3,i)], 'r','linewidth',2);
        plot3([p_w_c(1,i) camera_y(1,i)], [p_w_c(2,i) camera_y(2,i)], [p_w_c(3,i) camera_y(3,i)], 'r','linewidth',2);
        plot3([p_w_c(1,i) camera_z(1,i)], [p_w_c(2,i) camera_z(2,i)], [p_w_c(3,i) camera_z(3,i)], 'r','linewidth',2);

        % draw imu
        plot3([p_w_c(1,i) p_w_i(1,i)], [p_w_c(2,i) p_w_i(2,i)], [p_w_c(3,i) p_w_i(3,i)], 'c-', 'linewidth',2);
        
        axis equal; grid on; camproj('p');  
        axis([-10 30 -10 30 -10 10]); 
        xlabel('x'); ylabel('y'); zlabel('z');
        title(sprintf('Frame %d/%d', i, length(t)-1));
        view([-16 18]); %view([-41 36]);
        set(gca,'Color','k');
        hold off;
           
        subplot('position',[0.5,0.15,0.46,0.6]);
        scatter(observed_pts_c(1:2:end,i), observed_pts_c(2:2:end,i), 16, [.2,.5,0], 'filled');
        axis equal;
        axis([0 image_width 0 image_height]);
        xlabel('x'); ylabel('-y');
        title('Camera image');
        set(gca,'Color','k');
        %pause(0.01);        
        
    end
    
end