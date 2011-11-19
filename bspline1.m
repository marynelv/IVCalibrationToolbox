function P=bspline1(P0,P1,u,u0,u1)

if size(P0,2)==1, P0=repmat(P0,1,length(u)); end

P=bsxfun( @times,(u1-u)/(u1-u0),P0) + bsxfun( @times,(u0-u)/(u0-u1),P1);

end