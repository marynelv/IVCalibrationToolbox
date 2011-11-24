h=impoly; C3=getPosition(h);

%t=[0,1,2,3,4,5;
%    0,0,1,2,3,3;
%    0,0,1,2,3,4;
%    0,0,1,1,2,2;
%    0,1,2,3,3,3;
%    0,1,1,1,2,2;
%    0,1,1,2,2,3;
%    0,1,2,2,2,3];
%t=[0,0,1,1,2,2];
%t=[0,1,2,3,3,3;
%    0,1,2,2,2,3];

done=false;

while ~done
    
t=input(sprintf('enter %d knots: ', size(C3,1)+3-1));

done=t(1)==-1;

if ~done
for i=1:size(t,1)
    hold off;plot(C3(:,1),C3(:,2),'bo--');
    tt=mat2cell(t);
    title(sprintf(repmat('%d,',1,length(t)),tt{:}));
    for j=1:size(C3,1)-3
        pp=bspline3(C3(j:j+3,:)',t(j:j+5));
        hold on; plot(pp(1,:),pp(2,:),'g-');
    end
    drawnow;
    %pause;
    saveas(gca,sprintf('plots/plot3p%d.png',i));
end

end

end