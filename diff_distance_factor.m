function [diff_dist] = diff_distance_factor(cu, tu, n,flag)
x = cu(1,1);
y = cu(2,1);
if flag ==1 % changing the z based on the robot position if robot is near to obstracle it only moves in xy plane.
    z = 0;
    zt = 0;
else
    z = cu(3,1);
    zt = tu(3,1);
end
xt = tu(1,1);
yt = tu(2,1);
diff_dist = [abs(n*(x-xt)^(n-1)); abs(n*(y-yt)^(n-1)); abs(n*(z-zt)^(n-1))]; % differentiation of dist_factor
end