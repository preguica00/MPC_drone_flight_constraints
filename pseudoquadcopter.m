function dydt = quadcopter_ode(~,y)
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
% Unpack the state and input vectors

pitch= y(3);
velocity_x= y(4);
velocity_z= y(5);
velocity_pitch= y(6);
f1 = y(7);
f2 = y(8);

%Equations of motion
x_acceleration = (1/mass)*sin(pitch)* (f1+f2);
z_acceleration = gravitational_acceleration -(1/mass)*cos(pitch)* (f1+f2);
pitch_acceleration = (arm_moment/inertia_moment)*(f1-f2);

dydt(1)= velocity_x;
dydt(2)= velocity_z;
dydt(3) = velocity_pitch;
dydt(4) = x_acceleration;
dydt(5) = z_acceleration;
dydt(6)= pitch_acceleration;
% keep thrust rates constant here
dydt(7) = 0;
dydt(8) = 0;
end