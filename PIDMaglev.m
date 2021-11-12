%Function for PID Control for magnetic levitation
function dx = PIDMaglev(t, x)

k1 = 2.4831e-5; m = 0.02; R = 0.95; L = 0.27; g = 9.81; % Constants - k1 is Spring Constant, m is mass, R is Resistance
%L is inductance and g is gravitational constant. The above values are
%taken from research paper.

%xd is the desired state. We have 3 states in our system. Position of ball,
% Velocity of ball and Current. The desired position of the ball is 0.009m
% below our electromagnet, desired velocity is 0 and desired current is
% 0.8A. The last state is for integral control which is taken 0.
xd = [0.009; 0; 0.8; 0];  

K = [-2762.6 -58.7 25.8 -2000]; % Control gains for the states. There values are also taken from the research paper.

u = K * (xd - [x(1); x(2); x(3); x(4)]); % PID Control u = K * (Desired state - Current State)

%x1dot, x2dot and x3dot are the state space equations for our system and
%x4dot is for integral control.
x1dot = x(2);
x2dot = g - k1/m*x(3)^2/x(1)^2;
x3dot = -R / L * x(3) + 2 * k1 / L * x(2) * x(3) / x(1) ^ 2 + u / L;
x4dot = x(1) - xd(1);

dx = [x1dot; x2dot; x3dot; x4dot];


