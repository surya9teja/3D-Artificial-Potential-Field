% Function for creating cylinders 
% The follwing function is refered from the Dr Lyuba ALBOUL lecture notes.
function [h1, h2, h3] = create_cylinder(Radius, pos,color)
nSides = 100;
[X,Y,Z] = cylinder(Radius, nSides);
Height = pos(3);
Z = Z*Height;
Xpos = pos(1);
Ypos = pos(2);
X = X + Xpos;
Y = Y + Ypos;
h1 = surf(X,Y,Z,'facecolor',color,'LineStyle','none');
h2 = fill3(X(1,:),Y(1,:),Z(1,:),color);
h3 = fill3(X(2,:),Y(2,:),Z(2,:),color);
end


