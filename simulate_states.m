load ('states.mat')
load ('control.mat')


figure
subplot(3,2,1)
plot(state_trajectory(:,1), 'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('x (m)')
xlabel('Discrete Time')

subplot(3,2,2)
plot(state_trajectory(:,2),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('z (m)')
xlabel('Discrete Time')

subplot(3,2,3)
plot(rad2deg(state_trajectory(:,3)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('\theta (º)')
xlabel('Discrete Time')

subplot(3,2,4)
plot(state_trajectory(:,4),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{x}$ (m/s)', 'Interpreter','latex')
xlabel('Discrete Time')

subplot(3,2,5)
plot(state_trajectory(:,5),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{z}$ (m/s)', 'Interpreter','latex')
xlabel('Discrete Time')

subplot(3,2,6)
plot(rad2deg(state_trajectory(:,6)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{\theta}$ (m/s)', 'Interpreter','latex')
xlabel('Discrete Time')

figure
subplot(1,2,1)
plot(control_variables(:,1),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$u_1$ (m)', 'Interpreter','latex')
xlabel('Discrete Time')
subplot(1,2,2)
plot(control_variables(:,2),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$u_2$ (m)', 'Interpreter','latex')
xlabel('Discrete Time')
