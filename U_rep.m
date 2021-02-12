function [Urep] = U_rep(Krep, obstacle_pos, robo_pos, target, radius, flag)
zeta = 3*radius;
x = robo_pos(1,1);
y = robo_pos(2,1);
xo = obstacle_pos(1,1);
yo = obstacle_pos(2,1);
n = 2;
rou = sqrt((x-xo)^2+(y-yo)^2);
if rou<zeta
    Urep = 0.5*Krep*((1/rou)-(1/zeta))^2*dist_factor(robo_pos, target, n, flag);
else
    Urep = 0;
end
end