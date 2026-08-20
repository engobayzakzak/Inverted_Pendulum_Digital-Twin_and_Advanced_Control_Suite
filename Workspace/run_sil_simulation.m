%% =========================================================================
%  FILE: run_sil_simulation.m
%  PURPOSE: SIL Non-linear vs Linear LQR Step Response & Disturbance Test
% =========================================================================
system_params; % Load parameters and gains

% Initial Conditions: 10-degree initial tilt angle disturbance
x0 = [0.0; 0.0; deg2rad(10); 0.0]; 

% Simulation Settings
t_span = 0:0.001:5.0; % 5-second simulation at 1 kHz resolution
x_target = 0.50;      % 0.5m Cart Step Target

% ODE45 Non-linear closed-loop simulation
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_out, x_out] = ode45(@(t, x) closed_loop_ode(t, x, x_target, K_lqr, N_bar, params), t_span, x0, options);

% Reconstruct Control Force Trajectory
u_history = zeros(length(t_out), 1);
for i = 1:length(t_out)
    error_state = x_out(i,:)' - [0; 0; 0; 0];
    u_history(i) = -K_lqr * error_state + N_bar * x_target;
    % Actuator hard saturation clamp [-25N, +25N]
    u_history(i) = max(min(u_history(i), 25), -25);
end

%% PLOTTING PUBLICATION-GRADE SYSTEM METRICS
figure('Color', [1 1 1], 'Position', [100, 100, 1000, 750]);

subplot(3,1,1);
plot(t_out, x_out(:,1), 'LineWidth', 2.0, 'Color', [0 0.4470 0.7410]); hold on;
yline(x_target, '--r', 'Target x_{ref} = 0.5m', 'LineWidth', 1.5);
grid on; ylabel('Cart Position x [m]'); title('LQR Non-Linear Closed-Loop Benchmark');
set(gca, 'FontSize', 11);

subplot(3,1,2);
plot(t_out, rad2deg(x_out(:,3)), 'LineWidth', 2.0, 'Color', [0.8500 0.3250 0.0980]);
grid on; ylabel('Tilt Angle \theta [deg]'); yline(0, 'k--');
set(gca, 'FontSize', 11);

subplot(3,1,3);
plot(t_out, u_history, 'LineWidth', 1.8, 'Color', [0.4660 0.6740 0.1880]);
grid on; ylabel('Control Force F [N]'); xlabel('Time [seconds]');
yline(25, 'r:'); yline(-25, 'r:');
set(gca, 'FontSize', 11);

%% CLOSED-LOOP ODE HELPER
function dxdt = closed_loop_ode(t, x, x_ref, K, N_bar, params)
    % Control Law with Reference Feedforward: u(t) = -K*x(t) + N_bar*x_ref
    F = -K * x + N_bar * x_ref;
    
    % Hard Actuator Saturation modeling
    F = max(min(F, 25.0), -25.0);
    
    dxdt = nonlinear_dynamics(t, x, F, params);
end