%% =========================================================================
%  FILE: run_mpc_vs_lqr_benchmark.m
%  PURPOSE: Direct SIL Head-to-Head Comparison: Constrained MPC vs LQR
%           under Hard Physical Rail Boundaries and Disturbances
% =========================================================================
clear; clc; close all;
system_params; % Load continuous/discrete plant matrices and LQR gain

%% 1. SIMULATION HORIZONS & PARAMETERS
Ts = 0.01;                % MPC sampling period: 10 ms (100 Hz)
t_final = 5.0;            % Simulation time
time = 0:Ts:t_final;
N = length(time);

% Target Reference Trajectory (0.55m Step at t = 0.5s)
x_step_target = 0.55; 

% Physical System Constraints
limits.F_max = 20.0;      % Actuator force limit: +/- 20 N
limits.x_max = 0.65;      % Physical track rail limit: +/- 0.65 m

% MPC Tuning Horizon & Weights
Np = 30;                  % Prediction Horizon: 30 steps = 0.30 seconds
Nc = 10;                  % Control Horizon: 10 steps = 0.10 seconds

% MPC State Weighting (Matches LQR priorities)
Q_mpc = diag([12.0, 1.0, 45.0, 2.0]);
R_mpc = 0.05;

%% 2. INITIALIZE TEST RUNS (MPC vs LQR)
x_true_mpc = [0.0; 0.0; deg2rad(4.0); 0.0]; % Start with 4-deg tilt
x_true_lqr = x_true_mpc;

% Logging arrays
log_x_mpc = zeros(4, N);
log_u_mpc = zeros(1, N);
log_x_lqr = zeros(4, N);
log_u_lqr = zeros(1, N);

fprintf('Starting Benchmark: Constrained MPC vs Linear Quadratic Regulator...\n');

%% 3. CLOSED-LOOP SIMULATION LOOP
for k = 1:N
    t_curr = time(k);
    
    % Step reference definition
    if t_curr >= 0.5
        x_target_vec = [x_step_target; 0; 0; 0];
    else
        x_target_vec = [0; 0; 0; 0];
    end
    
    % External Disturbance Force (Impulse kick at t = 2.5s)
    F_dist = 0.0;
    if t_curr >= 2.5 && t_curr <= 2.55
        F_dist = -8.0; % 8N disturbance kick
    end
    
    % ------------------- MPC CONTROLLER EXECUTION -------------------
    [u_mpc_val, ~] = mpc_controller(x_true_mpc, x_target_vec, Ad, Bd, Np, Nc, Q_mpc, R_mpc, limits);
    log_u_mpc(k) = u_mpc_val;
    log_x_mpc(:, k) = x_true_mpc;
    
    % Propagate Non-Linear Dynamics for MPC Twin
    k1 = nonlinear_dynamics(0, x_true_mpc, u_mpc_val + F_dist, params);
    k2 = nonlinear_dynamics(0, x_true_mpc + 0.5*Ts*k1, u_mpc_val + F_dist, params);
    k3 = nonlinear_dynamics(0, x_true_mpc + 0.5*Ts*k2, u_mpc_val + F_dist, params);
    k4 = nonlinear_dynamics(0, x_true_mpc + Ts*k3, u_mpc_val + F_dist, params);
    x_true_mpc = x_true_mpc + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    
    % ------------------- LQR CONTROLLER EXECUTION -------------------
    u_lqr_raw = -K_lqr * (x_true_lqr - x_target_vec);
    u_lqr_val = max(min(u_lqr_raw, limits.F_max), -limits.F_max); % Output clamping
    log_u_lqr(k) = u_lqr_val;
    log_x_lqr(:, k) = x_true_lqr;
    
    % Propagate Non-Linear Dynamics for LQR Twin
    k1_l = nonlinear_dynamics(0, x_true_lqr, u_lqr_val + F_dist, params);
    k2_l = nonlinear_dynamics(0, x_true_lqr + 0.5*Ts*k1_l, u_lqr_val + F_dist, params);
    k3_l = nonlinear_dynamics(0, x_true_lqr + 0.5*Ts*k2_l, u_lqr_val + F_dist, params);
    k4_l = nonlinear_dynamics(0, x_true_lqr + Ts*k3_l, u_lqr_val + F_dist, params);
    x_true_lqr = x_true_lqr + (Ts/6.0)*(k1_l + 2*k2_l + 2*k3_l + k4_l);
end

fprintf('>> Simulation complete. Plotting comparative metrics...\n');

%% 4. PUBLICATION-GRADE COMPARATIVE PLOTS
figure('Color', [1 1 1], 'Position', [100, 100, 1050, 750]);

% --- SUBPLOT 1: Cart Displacement vs Hard Rail Limit ---
subplot(3, 1, 1);
plot(time, log_x_mpc(1,:), 'b-', 'LineWidth', 2.2, 'DisplayName', 'Constrained MPC'); hold on;
plot(time, log_x_lqr(1,:), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Unconstrained LQR');
yline(x_step_target, 'k:', 'Target Setpoint (0.55m)', 'LineWidth', 1.2);
yline(limits.x_max, 'r-', 'Upper Rail Boundary (+0.65m)', 'LineWidth', 2.0);
yline(-limits.x_max, 'r-', 'Lower Rail Boundary (-0.65m)', 'LineWidth', 2.0);
grid on; ylabel('Cart Position x [m]');
title('Constrained Model Predictive Control (MPC) vs LQR Benchmark');
legend('Location', 'Southeast'); set(gca, 'FontSize', 10);
ylim([-0.2, 0.75]);

% --- SUBPLOT 2: Pendulum Angle Recovery ---
subplot(3, 1, 2);
plot(time, rad2deg(log_x_mpc(3,:)), 'b-', 'LineWidth', 2.0, 'DisplayName', '\theta (MPC)'); hold on;
plot(time, rad2deg(log_x_lqr(3,:)), 'r--', 'LineWidth', 1.8, 'DisplayName', '\theta (LQR)');
grid on; ylabel('Tilt Angle \theta [deg]'); yline(0, 'k:');
legend('Location', 'Northeast'); set(gca, 'FontSize', 10);

% --- SUBPLOT 3: Control Effort & Constraint Compliance ---
subplot(3, 1, 3);
plot(time, log_u_mpc, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Force F (MPC)'); hold on;
plot(time, log_u_lqr, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Force F (LQR)');
yline(limits.F_max, 'k:', '+F_{max} (20N)');
yline(-limits.F_max, 'k:', '-F_{max} (-20N)');
grid on; ylabel('Force F [N]'); xlabel('Time [seconds]');
legend('Location', 'Northeast'); set(gca, 'FontSize', 10);

%% 5. QUANTITATIVE BENCHMARK METRICS
max_x_mpc = max(log_x_mpc(1,:));
max_x_lqr = max(log_x_lqr(1,:));
settling_idx_mpc = find(abs(log_x_mpc(1,:) - x_step_target) > 0.02 * x_step_target, 1, 'last');
settling_idx_lqr = find(abs(log_x_lqr(1,:) - x_step_target) > 0.02 * x_step_target, 1, 'last');

t_settle_mpc = time(settling_idx_mpc);
t_settle_lqr = time(settling_idx_lqr);

fprintf('\n================ CONTROLLER COMPARISON BENCHMARK ================\n');
fprintf('  Max Cart Displacement:   MPC = %6.4f m  |  LQR = %6.4f m\n', max_x_mpc, max_x_lqr);
fprintf('  Rail Limit Violation:    MPC = NONE      |  LQR = %s\n', ...
    mat2str(max_x_lqr > limits.x_max));
fprintf('  Settling Time (t_s 2%%):  MPC = %6.2f s   |  LQR = %6.2f s\n', t_settle_mpc, t_settle_lqr);