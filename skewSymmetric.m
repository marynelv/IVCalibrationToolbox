function [ skew_v ] = skewSymmetric( v )
%skewSymmetric Creates a skew symmetric matrix of the vector v
%   The function creates a 3-by-3 matrix which performs the function:
%    skewSymmetric(v)*w = cross(v,w)

assert( length(v) == 3 );

skew_v = [  0       -v(3)   v(2);
            v(3)    0       -v(1);
            -v(2)   v(1)    0];

end

