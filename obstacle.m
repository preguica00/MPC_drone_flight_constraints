function [xobs,yobs, obj_coord,radius] = obstacle
 radius = [7;1]; % [object radius; agent radius]   
obj_coord = [30.1;30]; % position of the obstacle
theta = 0 : 0.01 : 2*pi;
radius_o = radius(1,1);
xobs = radius_o * cos(theta) + obj_coord(1,1);
yobs = radius_o * sin(theta) + obj_coord(2,1);


end

