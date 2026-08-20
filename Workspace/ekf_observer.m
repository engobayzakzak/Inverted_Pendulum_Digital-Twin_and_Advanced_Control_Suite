function [x_hat_next, P_next, K_gain] = ekf_observer(x_hat, P, u, y_meas, Ts, Q_d, R_d, params)
% =========================================================================
% EKF_OBSERVER: Extended Kalman Filter for Inverted Pendulum State Estimation
% Inputs:
%   x_hat  : State estimate at k-1 [4x1]
%   P      : Error covariance matrix at k-1 [4x4]
%   u      : Control force applied at k-1 [1x1]
%   y_meas : Noisy sensor measurement at k [2x1] ([x; theta])
%   Ts     : Sampling time [s]
%   Q_d    : Discrete process noise covariance [4x4]
%   R_d    : Discrete measurement noise covariance [2x2]
%   params : System physical parameters struct
% Outputs:
%   x_hat_next : Posterior state estimate at k [4x1]
%   P_next     : Posterior error covariance at k [4x4]
%   K_gain     : Kalman gain matrix [4x2]
% =========================================================================

%% STEP 1: TIME UPDATE (STATE PREDICTION VIA RK4)
k1 = nonlinear_dynamics(0, x_hat, u, params);
k2 = nonlinear_dynamics(0, x_hat + 0.5*Ts*k1, u, params);
k3 = nonlinear_dynamics(0, x_hat + 0.5*Ts*k2, u, params);
k4 = nonlinear_dynamics(0, x_hat + Ts*k3, u, params);
x_hat_pred = x_hat + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);

%% STEP 2: TIME UPDATE (COVARIANCE PROPAGATION VIA NUMERICAL JACOBIAN)
% Evaluate continuous Jacobian F = df/dx at prior estimate
delta = 1e-6;
F = zeros(4, 4);
f0 = nonlinear_dynamics(0, x_hat, u, params);
for j = 1:4
    x_perturbed = x_hat;
    x_perturbed(j) = x_perturbed(j) + delta;
    f_perturbed = nonlinear_dynamics(0, x_perturbed, u, params);
    F(:, j) = (f_perturbed - f0) / delta;
end

% Discretize Jacobian transition matrix: Phi = expm(F*Ts) ≈ I + F*Ts
Phi = eye(4) + F * Ts;

% Predict error covariance: P_{k|k-1}
P_pred = Phi * P * Phi' + Q_d;

%% STEP 3: MEASUREMENT UPDATE (CORRECTION)
% Measurement model: y = [x; theta]
H = [1, 0, 0, 0;
     0, 0, 1, 0];

% Innovation (Residual)
y_pred = H * x_hat_pred;
y_residual = y_meas - y_pred;

% Innovation Covariance
S = H * P_pred * H' + R_d;

% Near-Optimal Kalman Gain
K_gain = (P_pred * H') / S;

% Posterior State Estimate Update
x_hat_next = x_hat_pred + K_gain * y_residual;

% Joseph Form Covariance Update (Guarantees Symmetry and Positive-Definiteness)
I_KH = eye(4) - K_gain * H;
P_next = I_KH * P_pred * I_KH' + K_gain * R_d * K_gain';
end