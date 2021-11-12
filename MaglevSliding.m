%Function for Sliding Mode Control for magnetic levitation
function dx = MaglevSliding(t, x)
k1 = 2.4831e-5; m = 0.02; R = 0.95; L = 0.27; g = 9.81; b = 0.202; % Constants - k1 is Spring Constant, m is mass, R is Resistance
%L is inductance and g is gravitational constant. The above values are
%taken from research paper.

%x1, x2 and x3 is the desired states for position, velocity and Current
%respectively
x1 = 0.009; x2 = 0; x3 = 0.8;
% A and B matrix are linearization matrices.
A = [0 1 0; 2 *k1/m*x3^2/x1^3 0 -2*k1/m*x3/x1^2; -4*k1 /L*x2*x3/x1^3 2*k1/L*x3/x1^2 -R/L + 2*k1/L*x2/x1^2];
B = [0; 0; 1/L];

%The sliding surface equation is s = G * x where G is [G1 1] and the value
%of G1 is chosen such that (A11 - A12 * G1) is Hurwitz. We use the place
%command in matlab to calculate G1.
A11 = A(1:2, 1:2);
A12 = A(1:2, 3);
G1 = place(A11, A12, [-3, -5]);
G = [G1, 1];   
s = G * x;
xd = [0.009 0 0.8]';

% The control of the system is given by u = ue + un where ue and un is
% calculated by the formulas given below.
ue = inv(G * B) * G * A * (xd - x);
un = -b * sign(s);
u = un + ue;
% x1dot, x2dot and x3dot are the state space equations of our system.
x1dot = x(2);
x2dot = g - k1/m * (x(3) - 0.8)^2 / (x(1) - 0.009)^2;
x3dot = -R/L * (x(3) - 0.8) + 2*k1/L*x(2)*(x(3) - 0.8)/(x(1) - 0.009)^2 + u/L;
dx = [x1dot; x2dot; x3dot];
