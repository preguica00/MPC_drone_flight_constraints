function dydt = quadcopter_ode(t,y,u,g)

[H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration_controller,gravitational_acceleration_model] = parameters;
%% Unpack the state and input vectors

    position_x= y(1);
    position_z= y(2);
    pitch= y(3);
    velocity_x= y(4);
    velocity_z= y(5);
    velocity_pitch= y(6);

    diff_mode  = u(1);
    common_mode = u(2);


%%Equations of motion
x_acceleration = (1/mass)*sin(pitch)* common_mode;
z_acceleration = g -(1/mass)*cos(pitch)* common_mode;
pitch_acceleration = (arm_moment/inertia_moment)*diff_mode;

%% 
%dydt(1)= velocity_x;
%dydt(2)= velocity_z;
%dydt(3) = velocity_pitch;
%dydt(4) = x_acceleration;
%dydt(5) = z_acceleration;
%dydt(6)= pitch_acceleration;

dydt=[velocity_x;velocity_z;velocity_pitch;x_acceleration;z_acceleration;pitch_acceleration];
end