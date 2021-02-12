%Environment code
clf;
close all;
clear;
% Defining Goal position of the UAV.
goal = [185,120,20];
% Defining intial position of the UAV.
start = [10,10,0];
% Buildings Position
Cpos = [70,50,60; 20,60,40; 60,90,60; 140,40,50; 180,190,60; 30,180,60;100,20,30; 30,110,20; 150,100,35; 70,160,40; 110,140,20];
figure; hold on
x = 0:4:200;
y = 0:4:200;
xlabel("x");
ylabel("y");
zlabel("z");
xlim([0 200]);
ylim([0 200]);
zlim([0 100]);
radius = [6;4;9;7;5;5;6;4;9;7;5]; % Radius of the buildings
create_cylinder(radius(1,1),Cpos(1,:),[0.25, 0.58, 0.96]) %[Radius, X-position, Y-position, Color]
create_cylinder(radius(2,1),Cpos(2,:),[0.25, 0.58, 0.96])
create_cylinder(radius(3,1),Cpos(3,:),[0.25, 0.58, 0.96])
create_cylinder(radius(4,1),Cpos(4,:),[0.25, 0.58, 0.96])
create_cylinder(radius(5,1),Cpos(5,:),[0.25, 0.58, 0.96])
create_cylinder(radius(6,1),Cpos(6,:),[0.25, 0.58, 0.96])
create_cylinder(radius(7,1),Cpos(7,:),[0.25, 0.58, 0.96]) %[Radius, X-position, Y-position, Color]
create_cylinder(radius(8,1),Cpos(8,:),[0.25, 0.58, 0.96])
create_cylinder(radius(9,1),Cpos(9,:),[0.25, 0.58, 0.96])
create_cylinder(radius(10,1),Cpos(10,:),[0.25, 0.58, 0.96])
create_cylinder(radius(11,1),Cpos(11,:),[0.25, 0.58, 0.96])
grid on;
text(start(1,1)-1, start(1,2), start(1,3)+2,"UAV Initial Position")
plot3(start(1,1), start(1,2), start(1,3),'MarkerSize',10,"Marker","*","Color","cyan")
text(goal(1,1), goal(1,2), goal(1,3)+2,"UAV Target")
plot3(goal(1,1), goal(1,2), goal(1,3),'-s','MarkerSize',10,'MarkerFaceColor','green')

%Path Planning
obstacles = transpose(Cpos);
iteration = 350;                         %Iterations count
current_pos = transpose(start);
goal = transpose(goal);
previous_pos = current_pos;             %Intialising Previous position of the UAV  
Krep = 0.1;                             %Gain factor of repulsive potential field
Katt = 0.04;                             
delta = 0;
data_points = zeros(iteration,3); % storing the iteration values positions of UAV
F = zeros(3,length(obstacles));
Urep = 0;
figure(1)
title('Path Planning of a UAV')
for i=1:iteration
    p_Fr = 0;
    robot_height = current_pos(3,1);
    goal_height = goal(3,1);
    flag = 0;
    Fatt = potential_attraction(Katt, current_pos, goal);
    for k = 1: length(obstacles)
        % Measuring the horizantal distance between UAV and centre axis of the building 
        rou = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2); 
        % differentiation of variable rou 
        d_rou = [current_pos(1,1)-obstacles(1,k); current_pos(2,1)-obstacles(2,k)]/rou; 
        
        % Threshold value to judge whether the UAV near to buidling or not?
        zeta = 3.5*radius(k,1);
        n = 2;
        if rou<=zeta
            if robot_height <= obstacles(3,k)
                % Flag - that tells the UAV to move in xy plane
                % no increment in height. 
                flag = 1;
                Frep1 = Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(current_pos, goal, n, flag)*d_rou;
                Frep2 = -(n/2)*Krep*((1/rou)-(1/zeta))^2*dist_factor(current_pos, goal, n-1, flag)*diff_distance_factor(current_pos, goal, n, flag);
                F(:,k) = vertcat(Frep1,0)+Frep2; 
                Fatt = Katt*[goal(1,1)-current_pos(1,1);goal(2,1)-current_pos(2,1); 0];
            else
                F(:,k) = 0;
            end
        elseif rou > zeta
            
            F(:,k) = 0;
        end
    end
    Frep = sum(F,2); % summation of all repulsive forces
    Ft = Fatt + Frep;
    if flag == 1
        flag = 0;
        % changing the 'z' axis based on the UAV position if robot is near to obstracle it only moves in xy plane.
        Ft(3,1) = 0; 
    end
    previous_pos = current_pos;
    current_pos = current_pos+Ft;
    data_points(i,:)=transpose(current_pos);
    title(sprintf('Iterations %d', i))
    % Plotting the UAV position in real time
    plot3(current_pos(1,1), current_pos(2,1), current_pos(3,1),"Marker","*","Color","black");
    pause(0.07)
    drawnow
end
%Plotting of graphs
for i = 1:length(obstacles)
    % Plotting other usefull plots to analyse the UAV behaviour
    potential_plots(x,y,obstacles(:,i));
end
