function dydt = quadcopterController(~,y)
position_x = y(1);
position_y = y(2);
pitch= y(3);
velocity_x= y(4);
velocity_z= y(5);
velocity_pitch= y(6);
f1 = y(7);
f2 = y(8);

% calculate the next plant
dydt = quadcopter_ode([],y);

% calculate the new thrust rates
dydt(7) = 1;
dydt(8) = 1;

end