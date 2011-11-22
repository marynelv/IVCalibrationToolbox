function P=generatePosBspline(landmarks,numpoints)

% Get a position spline for the camera given the landmarks
%
% Brings up a figure window in which you manually specify
% the control points for the camera's track
%
% P=generatePosSpline(landmarks, numpoints)
%
% landmarks: 3 x P set of landmark points
% numpoints: how many output points you'd like (default of 100)
% 
% P: 3 x numpoints set of output points

if ~exist('numpoints','var')
    numpoints=100;
end

meanz=mean(landmarks(3,:),2);

figure(20); subplot(1,2,1); plot(landmarks(1,:),landmarks(2,:),'b.');
mx=max(landmarks,[],2);
mn=min(landmarks,[],2);
axis([mn(1)-20,mx(1)+20,mn(2)-20,mx(2)+20]);
axis equal;

hold on;
done=false;
while ~done
    h=impoly; pos=getPosition(h);
    pos=pos';
    done=size(pos,2)>3;
    if ~done
        fprintf('Sorry, you need 4 points or more for a cubic spline :D\n');
    end
end

controlpts=pos;
controlpts=[controlpts;meanz*(1+.05*randn(1,size(controlpts,2)))];

P=cubicSpline(controlpts,numpoints);

hold on; 
plot(P(1,:),P(2,:),'g-');

subplot(1,2,2); plot3(landmarks(1,:),landmarks(2,:),landmarks(3,:),'b.'); hold on; 
plot3(P(1,:),P(2,:),P(3,:),'g-'); axis vis3d; axis equal;

end