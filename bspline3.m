function P=bspline3(Ps,u,npts)
% Helper function to generate a single segment of a cubic spline
% 
% P=bspline3(Pcontrol,knots,numpoints)
%
% Pcontrol=d x 4 set of control points
% numpoints = number of desired output points
% u = 1 x 6 vector of knots
%
% P=d x numpoints list of output points for the spline

P0=Ps(:,1); P1=Ps(:,2); P2=Ps(:,3); P3=Ps(:,4);
u0=u(1); u1=u(2); u2=u(3); u3=u(4); u4=u(5); u5=u(6);

if exist('npts','var') && ~isempty(npts)
    stepsize=(u3-u2)/(npts);
else
    stepsize=.01;
end


u=u2:stepsize:u3-stepsize;

P01=bspline1(P0,P1,u,u0,u3);
P12=bspline1(P1,P2,u,u1,u4);
P23=bspline1(P2,P3,u,u2,u5);
P02=bspline1(P01,P12,u,u1,u3);
P13=bspline1(P12,P23,u,u2,u4);
P=bspline1(P02,P13,u,u2,u3);

end