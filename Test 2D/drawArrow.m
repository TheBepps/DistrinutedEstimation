function [ output_args ] = drawArrow(startpoint,endpoint,headsize) 
% draw the communication direction between two agents
% accepts two [x y] coords and one double headsize 

v1 = headsize*(startpoint-endpoint)/sqrt((startpoint(1)-endpoint(1))^2+(startpoint(2)-endpoint(2))^2);
theta = 22.5*pi/180; 
theta1 = -1*22.5*pi/180; 
rotMatrix = [cos(theta)  -sin(theta) ; sin(theta)  cos(theta)];
rotMatrix1 = [cos(theta1)  -sin(theta1) ; sin(theta1)  cos(theta1)];  

v2 = v1*rotMatrix; 
v3 = v1*rotMatrix1;
x1 = endpoint+2*v1;
x2 = x1 + v2;
x3 = x1 + v3;

fill([x1(1) x2(1) x3(1)],[x1(2) x2(2) x3(2)],[0 0 0]);% this fills the arrowhead (black) 

plot([startpoint(1) endpoint(1)],[startpoint(2) endpoint(2)],'linewidth',0.5,'color',[0 0 0]);

end