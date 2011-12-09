cam_tl = [-.5  .5 1]';
cam_tr = [.5   .5 1]';
cam_bl = [-.5 -.5 1]';
cam_br = [.5  -.5 1]';
cam_img = [cam_tl cam_tr cam_bl cam_br].*2;

%% Movie like plot
if plotFlag
    for i = 1:length(t)-1
        
        if (mod(t(i), 1) == 0)          
            figure(9);
            
%             scatter3(p_w(1,1), p_w(2,1), p_w(3,1),50,'k','linewidth',2);
            plot3(p_w(1,:), p_w(2,:), p_w(3,:),'k-');
            hold on;
            
            scatter3(pts_w(1, :), pts_w(2, :), pts_w(3, :), 20, [.2,.5,0], 'filled');
            
            curR_w_c = T_w_i{i}*T_i_c;
            cam_coor = curR_w_c*[x_i.*2 y_i.*2 z_i.*2; ones(1,3)];
            
            plot3([curR_w_c(1,4) cam_coor(1,1)], [curR_w_c(2,4) cam_coor(2,1)], [curR_w_c(3,4) cam_coor(3,1)], 'r');            
            plot3([curR_w_c(1,4) cam_coor(1,2)], [curR_w_c(2,4) cam_coor(2,2)], [curR_w_c(3,4) cam_coor(3,2)], 'g');
%            plot3([curR_w_c(1,4) cam_coor(1,3)], [curR_w_c(2,4) cam_coor(2,3)], [curR_w_c(3,4) cam_coor(3,3)], 'b');
            
            cam_img_coor =  curR_w_c*[cam_img; ones(1,4)];
            plot3([curR_w_c(1,4) cam_img_coor(1,1)], [curR_w_c(2,4) cam_img_coor(2,1)], [curR_w_c(3,4) cam_img_coor(3,1)], 'b');            
            plot3([curR_w_c(1,4) cam_img_coor(1,2)], [curR_w_c(2,4) cam_img_coor(2,2)], [curR_w_c(3,4) cam_img_coor(3,2)], 'b');
            plot3([curR_w_c(1,4) cam_img_coor(1,3)], [curR_w_c(2,4) cam_img_coor(2,3)], [curR_w_c(3,4) cam_img_coor(3,3)], 'b');
            plot3([curR_w_c(1,4) cam_img_coor(1,4)], [curR_w_c(2,4) cam_img_coor(2,4)], [curR_w_c(3,4) cam_img_coor(3,4)], 'b');

            plot3([cam_img_coor(1,2) cam_img_coor(1,1)], [cam_img_coor(2,2) cam_img_coor(2,1)], [cam_img_coor(3,2) cam_img_coor(3,1)], 'b');            
            plot3([cam_img_coor(1,4) cam_img_coor(1,2)], [cam_img_coor(2,4) cam_img_coor(2,2)], [cam_img_coor(3,4) cam_img_coor(3,2)], 'b');
            plot3([cam_img_coor(1,3) cam_img_coor(1,4)], [cam_img_coor(2,3) cam_img_coor(2,4)], [cam_img_coor(3,3) cam_img_coor(3,4)], 'b');
            plot3([cam_img_coor(1,1) cam_img_coor(1,3)], [cam_img_coor(2,1) cam_img_coor(2,3)], [cam_img_coor(3,1) cam_img_coor(3,3)], 'b');

            
            hold off
            axis equal; grid on;
            xlabel('x');
            ylabel('y');
            zlabel('z');
            view([-223 -16]);
            h_xlabel = get(gca,'XLabel');
            set(h_xlabel,'FontSize',24); 
            h_ylabel = get(gca,'YLabel');
            set(h_ylabel,'FontSize',24); 
            h_zlabel = get(gca,'ZLabel');
            set(h_zlabel,'FontSize',24); 
            
            refresh
            pause(0.1)
        end
    end
end