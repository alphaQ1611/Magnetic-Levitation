clc; clear all; close all;
%Here we use ode45 solver to solve the PIDMaglev function.
[t, x] = ode45(@MaglevSliding, [0, 0.35], [0.0125 0 1.111]);
%The below code upto line 22 is done to calculate the values of control
%effort.
A11 = [0 1; 2180 0];
A12 = [0; -24.525];
G1 = place(A11, A12, [-3, -5]);
G = [G1 1];
k1 = 2.4831e-5; m = 0.02; R = 0.95; L = 0.27; g = 9.81; b = 0.202;
x1 = 0.009; x2 = 0; x3 = 0.8;
A = [0 1 0;2*k1/m*x3^2/x1^3 0 -2*k1/m*x3/x1^2;-4*k1/L*x2*x3/x1^3 2*k1/L*x3/x1^2 -R/L+2*k1/L*x2/x1^2];
B = [0; 0; 1 / L];
u = zeros(length(t),1);
for i = 1 : length(t)
    s = G * x(i, :)';
    ue = -inv(G * B) * G * A * x (i, :)';
    un = -b * sign(s);
    u(i) = un + ue;
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
