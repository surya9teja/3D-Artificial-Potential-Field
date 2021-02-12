%Potential reflection
function [Frep] = potential_repulsive(Krep, obstacle_pos,robo_pos, target, radius)
zeta = 3*radius;
x = robo_pos(1,1);
y = robo_pos(2,1);
xo = obstacle_pos(1,1);
yo = obstacle_pos(2,1);
n = 2;
rou = sqrt((x-xo)^2+(y-yo)^2);
d_rou = [robo_pos(1,1)-obstacle_pos(1,1); robo_pos(2,1)-obstacle_pos(2,1); 0];
if(rou<=zeta)
    Frep1 = Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(robo_pos, target, n)*d_rou;
    Frep2 = -(n/2)*Krep*((1/rou)-(1/zeta))^2*dist_factor(robo_pos, target, n-1)*diff_distance_factor(robo_pos, target, n);
    Frep = Frep1+Frep2+[0;0;0];
else
    Frep = 0;
end
end

