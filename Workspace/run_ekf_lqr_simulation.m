%% =========================================================================
%  FILE: run_ekf_lqr_simulation.m
%  PURPOSE: Full SIL Closed-Loop Benchmark of Non-linear Plant + EKF + LQR
% =========================================================================
clear; clc; close all;
system_params; % Load system matrices, LQR gain K, and physical parameters

%% 1. SIMULATION TIMING & PARAMETERS
Ts = 0.005;               % EKF & Control loop rate: 5 ms (200 Hz)
t_final = 5.0;            % Total simulation duration: 5.0 seconds
time = 0:Ts:t_final;
N = length(time);

% Target Reference Trajectory (0.5m Cart Step at t = 1.0s)
x_ref = zeros(1, N);
x_ref(time >= 1.0) = 0.50;

%% 2. STOCHASTIC NOISE COVARIANCE DEFINITIONS
% Discrete Process Noise Q_d (disturbances on state derivatives)
sigma_w_pos = 0.001;      % Cart position drift [m]
sigma_w_vel = 0.01;       % Cart velocity disturbance [m/s]
sigma_w_ang = deg2rad(0.05); % Pendulum angle drift [rad]
sigma_w_omg = deg2rad(0.50); % Pendulum velocity disturbance [rad/s]
Q_d = diag([sigma_w_pos^2, sigma_w_vel^2, sigma_w_ang^2, sigma_w_omg^2]);

% Discrete Measurement Noise R_d (Sensor uncertainty)
sigma_meas_x = 0.005;     % Linear encoder accuracy: +/- 5mm
sigma_meas_th = deg2rad(0.35); % Angle sensor jitter: +/- 0.35 deg (~6 mrad)
R_d = diag([sigma_meas_x^2, sigma_meas_th^2]);

%% 3. INITIAL STATE CONDITIONS
% True Non-linear Plant State: [x; x_dot; theta; theta_dot]
% System starts with a 6-degree initial tilt angle error
x_true = [0.0; 0.0; deg2rad(6.0); 0.0];

% EKF Initial Guess (with initial state estimation uncertainty)
x_hat = [0.0; 0.0; 0.0; 0.0];
P_cov = diag([0.01, 0.1, deg2rad(5)^2, deg2rad(20)^2]);

%% 4. PRE-ALLOCATION FOR LOGGING
log_x_true = zeros(4, N);
log_x_hat  = zeros(4, N);
log_y_meas = zeros(2, N);
log_u      = zeros(1, N);
log_sigma  = zeros(4, N); % 3-Sigma Estimation Covariance Bounds

%% 5. CLOSED-LOOP REAL-TIME SIMULATION LOOP
fprintf('Starting Non-linear SIL Closed-Loop Simulation with EKF & LQR...\n');

u_k = 0.0;
for k = 1:N
    % 1. Measure True State with Additive Sensor Gaussian Noise
    v_k = [normrnd(0, sigma_meas_x); 
           normrnd(0, sigma_meas_th)];
    y_sensor = [x_true(1); x_true(3)] + v_k;
    
    % 2. Extended Kalman Filter (EKF) State Observer Update
    [x_hat, P_cov, ~] = ekf_observer(x_hat, P_cov, u_k, y_sensor, Ts, Q_d, R_d, params);
    
    % 3. LQR State Feedback Control Law: u = -K * x_hat + N_bar * r(t)
    r_k = x_ref(k);
    u_raw = -K_lqr * x_hat + N_bar * r_k;
    
    % Actuator Hard Saturation Modeling [-25N, +25N]
    u_k = max(min(u_raw, 25.0), -25.0);
    
    % 4. Log Real-Time Signals
    log_x_true(:, k) = x_true;
    log_x_hat(:, k)  = x_hat;
    log_y_meas(:, k) = y_sensor;
    log_u(k)         = u_k;
    log_sigma(:, k)  = 3 * sqrt(diag(P_cov)); % Extract 3-sigma bounds
    
    % 5. Propagate True Non-Linear Physics Twin (with process noise)
    w_k = [normrnd(0, sigma_w_pos);
           normrnd(0, sigma_w_vel);
           normrnd(0, sigma_w_ang);
           normrnd(0, sigma_w_omg)];
       
    % RK4 High-fidelity continuous physics propagation
    k1 = nonlinear_dynamics(0, x_true, u_k, params);
    k2 = nonlinear_dynamics(0, x_true + 0.5*Ts*k1, u_k, params);
    k3 = nonlinear_dynamics(0, x_true + 0.5*Ts*k2, u_k, params);
    k4 = nonlinear_dynamics(0, x_true + Ts*k3, u_k, params);
    x_true = x_true + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4) + w_k;
end

fprintf('>> Simulation completed successfully.\n');

%% 6. COMPREHENSIVE PERFORMANCE & ESTIMATION ACCURACY VISUALIZATION
figure('Color', [1 1 1], 'Position', [80, 50, 1100, 850]);

% --- SUBPLOT 1: Cart Position Tracking & Estimation ---
subplot(4, 1, 1);
plot(time, log_x_true(1,:), 'k-', 'LineWidth', 2.0, 'DisplayName', 'True State x'); hold on;
plot(time, log_x_hat(1,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate \hat{x}');
plot(time, log_y_meas(1,:), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 4, 'DisplayName', 'Noisy Sensor Measurement');
plot(time, x_ref, 'r:', 'LineWidth', 1.8, 'DisplayName', 'Setpoint Target');
grid on; ylabel('Position [m]'); title('Inverted Pendulum Digital Twin: Closed-Loop EKF + LQR Control');
legend('Location', 'Southeast'); set(gca, 'FontSize', 10);

% --- SUBPLOT 2: Pendulum Angle Stabilization ---
subplot(4, 1, 2);
plot(time, rad2deg(log_x_true(3,:)), 'k-', 'LineWidth', 2.0, 'DisplayName', 'True Angle \theta'); hold on;
plot(time, rad2deg(log_x_hat(3,:)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate \hat{\theta}');
plot(time, rad2deg(log_y_meas(2,:)), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 4, 'DisplayName', 'Noisy Angle Sensor');
grid on; ylabel('Angle \theta [deg]'); legend('Location', 'Northeast'); set(gca, 'FontSize', 10);

% --- SUBPLOT 3: Estimation Error & 3-Sigma Filter Convergence Bounds ---
subplot(4, 1, 3);
err_theta = rad2deg(log_x_true(3,:) - log_x_hat(3,:));
plot(time, err_theta, 'm-', 'LineWidth', 1.2, 'DisplayName', 'Angle Error e_{\theta}'); hold on;
plot(time,  rad2deg(log_sigma(3,:)), 'k--', 'LineWidth', 1.2, 'DisplayName', '+3\sigma Bound');
plot(time, -rad2deg(log_sigma(3,:)), 'k--', 'LineWidth', 1.2, 'DisplayName', '-3\sigma Bound');
grid on; ylabel('e_{\theta} Error [deg]'); legend('Location', 'Northeast'); set(gca, 'FontSize', 10);
title('EKF Observer Consistency (Error trapped inside 3\sigma Covariance Envelope)');

% --- SUBPLOT 4: Optimal Actuator Control Force Output ---
subplot(4, 1, 4);
plot(time, log_u, 'Color', [0 0.5 0], 'LineWidth', 1.6, 'DisplayName', 'LQR Force F [N]'); hold on;
yline(25, 'r:', 'Saturation Limit (+25N)'); yline(-25, 'r:', 'Saturation Limit (-25N)');
grid on; ylabel('Force F [N]'); xlabel('Time [seconds]'); legend('Location', 'Northeast'); set(gca, 'FontSize', 10);

%% 7. QUANTITATIVE STATE ESTIMATION METRICS
rmse_x     = sqrt(mean((log_x_true(1,:) - log_x_hat(1,:)).^2));
rmse_xdot  = sqrt(mean((log_x_true(2,:) - log_x_hat(2,:)).^2));
rmse_theta = sqrt(mean((rad2deg(log_x_true(3,:) - log_x_hat(3,:))).^2));
rmse_thdot = sqrt(mean((rad2deg(log_x_true(4,:) - log_x_hat(4,:))).^2));

fprintf('\n================ EKF OBSERVER ESTIMATION ACCURACY (RMSE) ================\n');
fprintf('  Cart Position RMSE (x):             %6.4f m\n', rmse_x);
fprintf('  Cart Velocity RMSE (x_dot):         %6.4f m/s\n', rmse_xdot);
fprintf('  Pendulum Angle RMSE (theta):        %6.4f deg\n', rmse_theta);
fprintf('  Pendulum Angular Vel RMSE (th_dot): %6.4f deg/s\n', rmse_thdot);