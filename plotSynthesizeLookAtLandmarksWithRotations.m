%% Plot
if plotFlag
    for i = 1:length(t) - 1
        
        figure(1);
        grid on;

        % plot points
        subplot(1,2,1);
        scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 'r', '.');
        hold on;
        scatter3(pts_center(1), pts_center(2), pts_center(3), 'b', '.');
        
        
%         plot3([p_w_c(1,i) pts_center(1)], [p_w_c(2,i) pts_center(2)], [p_w_c(3,i) pts_center(3)],'m-'); 
        
        % plot camera path
        plot3(p_w_c(1,:), p_w_c(2,:), p_w_c(3,:), 'k-');
        
        % draw camera axis
        plot3([p_w_c(1,i) camera_x(1,i)], [p_w_c(2,i) camera_x(2,i)], [p_w_c(3,i) camera_x(3,i)], 'r');
        plot3([p_w_c(1,i) camera_y(1,i)], [p_w_c(2,i) camera_y(2,i)], [p_w_c(3,i) camera_y(3,i)], 'g');
        plot3([p_w_c(1,i) camera_z(1,i)], [p_w_c(2,i) camera_z(2,i)], [p_w_c(3,i) camera_z(3,i)], 'b');

        % draw imu
        plot3([p_w_c(1,i) p_w_i(1,i)], [p_w_c(2,i) p_w_i(2,i)], [p_w_c(3,i) p_w_i(3,i)], 'c-');
        
        axis equal; axis vis3d;    
        axis([-10 30 -5 45 -30 30]);
        xlabel('x'); ylabel('y'); zlabel('z');
        title(sprintf('frame %d/%d', i, length(t)-1));
        view([-16 18]); %view([-41 36]);
        hold off;
           
        subplot(1,2,2);
        scatter(observed_pts_c(1:2:end,i), observed_pts_c(2:2:end,i), 'r');
        axis equal;
        axis([0 image_width 0 image_height]);
        xlabel('x'); ylabel('-y');
        title('Camera image');
        
        %pause(0.01);        
        
    end
    
end