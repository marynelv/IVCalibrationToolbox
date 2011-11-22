function P=cubicSpline(Pcontrol, numpoints, knots)

% Cubic spline generator
% 
% P=bspline3(Pcontrol,numpoints,knots)
%
% Pcontrol=d x n set of control points
% numpoints = number of desired output points
% u = 1 x (n+2) vector of knots
%
% P=d x numpoints list of output points for the spline


ncontrol=size(Pcontrol,2);
nknots=ncontrol+2;
if ~exist('knots','var')
    knots=[0,0,0:nknots-5,nknots-5,nknots-5];
end

numpts=floor(numpoints/sum(diff(knots)));
numptss=repmat(numpts,1,ncontrol-3);
numptss(end)=numptss(end)+numpoints-sum(numptss);

P=[];

for i=1:ncontrol-3
    P=[P,bspline3(Pcontrol(:,i:i+3),knots(i:i+5),numptss(i))];
end

end