clear; clc; close all;
%% 1. Nominal Physics Parameters
params.M = 1.50; % Cart Mess [kg]
params.m = 0.25; % Pendulum Mass [kg]
params.l = 0.35;      % Distance to pendulum COM [m] (Total rod length = 0.70m)
params.I = (1/12) * params.m * (2*params.l)^2; % Moment of inertia about COM [kg*m^2]
params.b = 0.10;      % Cart viscous damping coefficient [N*s/m]
params.c = 0.005;     % Pendulum pivot damping coefficient [N*m*s/rad]
params.g = 9.81;      % Gravitational acceleration [m/s^2]

% Mass matrix determinant at upright equilibrium (theta = 0)
D0 = params.I * (params.M + params.m) + params.M * params.m * (params.l^2);
params.D0 = D0;

%% 2. CONTINUOUS-TIME STATE-SPACE MATRICES (LINEARIZED ABOUT x0 = [0,0,0,0]^T)
% State Vector: x = [cart_position (x), cart_velocity (x_dot), 
%                    pendulum_angle (theta), pendulum_angular_velocity (theta_dot)]^T
% Input: u = Horizontal Force F [N]

A = [0,                         1,                                                 0,                                            0;
     0, -(params.I + params.m*params.l^2)*params.b / D0, (params.m^2 * params.g * params.l^2) / D0,       -(params.m * params.l * params.c) / D0;
     0,                         0,                                                 0,                                            1;
     0,  (params.m * params.l * params.b) / D0,          (params.M + params.m)*params.m*params.g*params.l / D0, -(params.M + params.m)*params.c / D0];

B = [0;
     (params.I + params.m*params.l^2) / D0;
     0;
     -(params.m * params.l) / D0];

% Output Vector: y = [x, theta]^T (Direct sensor measurements)
C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = [0;
     0];

% Full State Output for State Feedback
C_full = eye(4);
D_full = zeros(4, 1);

plant_ss = ss(A, B, C, D);
plant_ss.StateName  = {'x', 'x_dot', 'theta', 'theta_dot'};
plant_ss.InputName  = {'F'};
plant_ss.OutputName = {'x', 'theta'};

%% 3. STRUCTURAL PROPERTIES: CONTROLLABILITY & OBSERVABILITY
% Controllability Matrix: C_mat = [B, AB, A^2*B, A^3*B]
Ctrb_mat = ctrb(A, B);
rank_ctrb = rank(Ctrb_mat);
cond_ctrb = cond(Ctrb_mat);

% Observability Matrix (under position + angle sensing): O_mat = [C; CA; CA^2; CA^3]
Obsv_mat = obsv(A, C);
rank_obsv = rank(Obsv_mat);
cond_obsv = cond(Obsv_mat);

fprintf('================ SYSTEM STRUCTURAL ANALYSIS ================\n');
fprintf('Controllability Matrix Rank: %d / 4 (Condition Number: %.2e)\n', rank_ctrb, cond_ctrb);
fprintf('Observability Matrix Rank:   %d / 4 (Condition Number: %.2e)\n', rank_obsv, cond_obsv);

if rank_ctrb == 4 && rank_obsv == 4
    fprintf('>> SYSTEM STATUS: Completely Controllable and Observable.\n');
else
    error('System is structurally deficient!');
end

%% 4. OPEN-LOOP POLE-ZERO & SPECTRUM ANALYSIS
open_loop_poles = eig(A);
fprintf('\nOpen-Loop Eigenvalues (Poles):\n');
for i = 1:length(open_loop_poles)
    fprintf('  p%d = %+.4f %+.4fj\n', i, real(open_loop_poles(i)), imag(open_loop_poles(i)));
end

% Transfer function representations
[num_x, den_x] = ss2tf(A, B, C(1,:), D(1));
[num_th, den_th] = ss2tf(A, B, C(2,:), D(2));

G_cart = minreal(tf(num_x, den_x));
G_pend = minreal(tf(num_th, den_th));

%% 5. OPTIMAL CONTROL SYNTHESIS: LQR CONTROLLER (CARE SOLVER)
% Design Strategy: Bryson's Rule with Engineering Penalty Weighting
% Max allowable errors:
max_x     = 0.50;         % Maximum Cart Displacement [m]
max_xdot  = 1.00;         % Maximum Cart Velocity [m/s]
max_theta = deg2rad(12);  % Maximum Pendulum Deviation [rad] (~0.209 rad)
max_thdot = deg2rad(60);  % Maximum Angular Velocity [rad/s] (~1.047 rad/s)
max_force = 15.0;         % Maximum Actuator Force [N]

% State Weighting Matrix Q and Control Weighting Matrix R
Q = diag([1/(max_x^2), 1/(max_xdot^2), 10/(max_theta^2), 1/(max_thdot^2)]);
R = 1 / (max_force^2);

% Solve the Continuous Algebraic Riccati Equation:
% A^T * P + P * A - P * B * R^(-1) * B^T * P + Q = 0
[K_lqr, P_care, cl_poles_lqr] = lqr(A, B, Q, R);

fprintf('\n================ OPTIMAL LQR CONTROLLER SYNTHESIS ================\n');
fprintf('LQR State Feedback Gain Matrix K:\n');
fprintf('  K_x        = %+.4f N/m\n', K_lqr(1));
fprintf('  K_x_dot    = %+.4f N/(m/s)\n', K_lqr(2));
fprintf('  K_theta    = %+.4f N/rad\n', K_lqr(3));
fprintf('  K_th_dot   = %+.4f N/(rad/s)\n', K_lqr(4));

fprintf('\nClosed-Loop Poles (A - B*K):\n');
for i = 1:length(cl_poles_lqr)
    fprintf('  lambda%d = %+.4f %+.4fj\n', i, real(cl_poles_lqr(i)), imag(cl_poles_lqr(i)));
end

% Pre-compensator Gain N_bar for Zero Steady-State Tracking of Cart Position:
% r(t) -> Cart Reference Position
A_cl = A - B * K_lqr;
% Compute feedforward gain: N_bar = -1 / (C_x * (A - B*K)^(-1) * B)
C_x = [1, 0, 0, 0];
N_bar = -inv(C_x * inv(A_cl) * B);
fprintf('\nFeedforward Pre-compensator Gain N_bar: %.4f\n', N_bar);

%% 6. DISCRETIZATION FOR DIGITAL IMPLEMENTATION (EKF & MPC)
Ts = 0.005; % Sampling period: 5 ms (200 Hz control loop)
plant_d = c2d(plant_ss, Ts, 'zoh');
Ad = plant_d.A;
Bd = plant_d.B;
Cd = plant_d.C;
Dd = plant_d.D;

% Save baseline parameters to workspace
save('workspace_vars.mat', 'params', 'A', 'B', 'C', 'D', 'K_lqr', 'N_bar', 'Q', 'R', 'Ad', 'Bd', 'Ts');
fprintf('\nWorkspace initialized successfully. Variables cached.\n');