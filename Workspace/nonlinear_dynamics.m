function x_dot = nonlinear_dynamics(t, x_state, F, params)
% =========================================================================
% NONLINEAR_DYNAMICS: Exact Equations of Motion for Inverted Pendulum
% State: x_state = [x; x_dot; theta; theta_dot]
% =========================================================================
x         = x_state(1);
x_dot     = x_state(2);
theta     = x_state(3);
theta_dot = x_state(4);

% Physical parameters
M = params.M;
m = params.m;
l = params.l;
I = params.I;
b = params.b;
c = params.c;
g = params.g;

% State-dependent mass matrix determinant
sin_th = sin(theta);
cos_th = cos(theta);
D_theta = (M + m)*(I + m*l^2) - (m*l*cos_th)^2;

% Non-linear Coriolis, Centrifugal, Gravity, and Friction Forces
% Eq 1: (M + m)*x_ddot + m*l*cos(theta)*theta_ddot - m*l*theta_dot^2*sin(theta) + b*x_dot = F
% Eq 2: (I + m*l^2)*theta_ddot + m*l*cos(theta)*x_ddot + c*theta_dot - m*g*l*sin(theta) = 0

term1 = F - b*x_dot + m*l*(theta_dot^2)*sin_th;
term2 = m*g*l*sin_th - c*theta_dot;

% Explicit acceleration solutions (Cramer's Rule)
x_ddot = ((I + m*l^2)*term1 - m*l*cos_th*term2) / D_theta;
theta_ddot = ((M + m)*term2 - m*l*cos_th*term1) / D_theta;

% First-order state derivative output
x_dot = [x_dot;
         x_ddot;
         theta_dot;
         theta_ddot];
end