% Rectangular joint axis coordinates to spherical

function [p1,p2,t1,t2] = RecToSphere(J)
    % Assign values to the j-vectors:
    j1_ = J(1:3);       j2_ = J(4:6);
    
    p1 = asin(j1_(3));
    t1 = acos( j1_(1) / (cos(asin(j1_(3)))));
    
    p2 = asin(j2_(3));
    t2 = acos( j2_(1) / (cos(asin(j2_(3)))));
end