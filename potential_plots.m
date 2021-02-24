%Credits Dr Lyuba Alboul's lecture notes

%Potting function
function [plots] = potential_plots(x,y, obstacle)
[xx, yy] = meshgrid(x,y);

% Measuring the distance between two points
pos1 = sqrt((xx-obstacle(1,1)).^2+(yy-obstacle(2,1)).^2);

% The following code produces a mesh graph that provides spikes and downs
% based on the building positions and goal position only ploted it while
% taking into the account of XY plane of the created environment.

for i = 1:numel(x)
    for j = 1:numel(y)
        if pos1(i,j) > 1.5 && pos1(i,j)<=5
            posf1(i,j)= pos1(i,j).^(-1);
        end
        if pos1(i,j)>= 5
            posf1(i,j) =0;    
        end
        if pos1(i,j) <=1.5
            posf1(i,j)= 1/(1.5);
        end  
    end
end
negd2=(xx-102).^2 +(yy-105).^2 -0.002;   
negd=sqrt(negd2);
neg=negd.^(-1);
zz=posf1-neg;
figure(2)
title('Potential Fields plot')
xlabel('x')
ylabel('y')
hold on
% A 3-D mesh plot of the potential
mesh(real(zz)) 
pause(1)				
figure(3)
title('Quiver plot of the environment')
xlabel('x')
ylabel('y')
hold on
[px,py]=gradient(zz,.1,.1);
quiver(x,y,-px,-py,1.2, 'r'), hold on
quiver(x(12),y(12),-px(12),-py(12),2, 'g')
pause(1)
figure(4)
title('Contour plot of the environment')
xlabel('x')
ylabel('y')
hold on
contour(zz,21)
end

