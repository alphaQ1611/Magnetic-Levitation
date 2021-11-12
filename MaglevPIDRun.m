clc; clear all; close all;

%Here we use ode45 solver to solve the PIDMaglev function.
[t, x] = ode45(@PIDMaglev, [0, 2], [0.0125 0 1.111 0]);

%The below code upto line 13 is done to calculate the values of control
%effort.
xd = [0.009; 0; 0.8; 0];
K = [-2762.6 -58.7 25.8 -2000];
u = zeros(length(t), 1);
for i = 1:length(t)
    u(i) = K * (xd - [x(i, 1); x(i, 2); x(i, 3); x(i, 4)]);
end

%Plotting graphs for position, Current and Control effort.
figure(1)
plot(t, x(:, 1))
xlabel('time (sec)'); ylabel('Position (m)')
grid on
figure(2)
plot(t, x(:, 3))
xlabel('time (sec)'); ylabel('Current (A)')
grid on
figure(3)
plot(t, u)
grid on
xlabel('time (sec)'); ylabel('Control Input')



