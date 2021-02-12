function [d] = dist_factor(cu, tu, n,flag)
x = cu(1,1);
y = cu(2,1);
% changing the z based on the robot position if robot is near to obstracle it only moves in xy plane.
if flag == 1 
    z = 0;
    zt = 0;
else
    z = cu(3,1);
    zt = tu(3,1);
end
xt = tu(1,1);
yt = tu(2,1);
d = abs((x-xt)^n)+abs((y-yt)^n)+abs((z-zt)^n);
end