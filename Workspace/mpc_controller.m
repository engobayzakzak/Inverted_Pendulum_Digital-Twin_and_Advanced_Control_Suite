function [u_opt, exitflag] = mpc_controller(x_curr, x_target, Ad, Bd, Np, Nc, Q_mpc, R_mpc, limits)
% =========================================================================
% MPC_CONTROLLER: Constrained Quadratic-Program (QP) Receding Horizon Solver
% Inputs:
%   x_curr   : Current state vector [4x1] [x; x_dot; theta; theta_dot]
%   x_target : Target state setpoint [4x1]
%   Ad, Bd   : Discrete system matrices
%   Np, Nc   : Prediction and Control horizons
%   Q_mpc    : State penalty matrix [4x4]
%   R_mpc    : Control effort penalty [scalar]
%   limits   : Struct containing constraint bounds:
%              limits.F_max (Actuator limit [N])
%              limits.x_max (Cart rail limit [m])
% Outputs:
%   u_opt    : First control input to apply u_k [1x1]
%   exitflag : QP solver status (1 = Converged to optimal solution)
% =========================================================================

nx = size(Ad, 1);
nu = size(Bd, 2);

%% 1. BUILD BATCH PREDICTION MATRICES (Sx and Su)
Sx = zeros(nx * Np, nx);
Su = zeros(nx * Np, nu * Nc);

% Compute Sx
A_power = eye(nx);
for i = 1:Np
    A_power = A_power * Ad;
    Sx((i-1)*nx + 1 : i*nx, :) = A_power;
end

% Compute Su
for i = 1:Np
    for j = 1:Nc
        if i >= j
            A_diff = Ad^(i - j);
            Su((i-1)*nx + 1 : i*nx, (j-1)*nu + 1 : j*nu) = A_diff * Bd;
        end
    end
end

%% 2. FORMULATE QUADRATIC COST MATRICES (Hessian H_qp and Gradient f_qp)
Q_bar = kron(eye(Np), Q_mpc);
R_bar = kron(eye(Nc), R_mpc);

% Stack reference trajectory across horizon
R_traj = repmat(x_target, Np, 1);

% QP Objective: (1/2)*U'*H_qp*U + f_qp'*U
H_qp = 2 * (Su' * Q_bar * Su + R_bar);
% Ensure symmetric Hessian for numerical stability
H_qp = (H_qp + H_qp') / 2;

f_qp = 2 * Su' * Q_bar * (Sx * x_curr - R_traj);

%% 3. ENFORCE HARD CONSTRAINTS
% Constraint 1: Actuator Force Bounds [-F_max, +F_max]
A_u = [eye(Nc); -eye(Nc)];
b_u = [repmat(limits.F_max, Nc, 1); repmat(limits.F_max, Nc, 1)];

% Constraint 2: State Rail Bounds [-x_max <= x <= x_max]
% Extract cart position row (state 1) across the prediction horizon
C_pos = [1, 0, 0, 0];
C_pos_bar = kron(eye(Np), C_pos);

Sx_pos = C_pos_bar * Sx;
Su_pos = C_pos_bar * Su;

A_x = [Su_pos; -Su_pos];
b_x = [repmat(limits.x_max, Np, 1) - Sx_pos * x_curr;
       repmat(limits.x_max, Np, 1) + Sx_pos * x_curr];

% Assemble complete linear inequality constraints: A_ineq * U <= b_ineq
A_ineq = [A_u; A_x];
b_ineq = [b_u; b_x];

%% 4. SOLVE QUADRATIC PROGRAM (QUADPROG)
options = optimoptions('quadprog', ...
                       'Display', 'off', ...
                       'Algorithm', 'interior-point-convex', ...
                       'MaxIterations', 100);

[U_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], [], [], [], options);

if exitflag == 1
    % Apply Receding Horizon Principle: apply only the first control action
    u_opt = U_opt(1);
else
    % Fallback: if QP infeasible, saturate at boundary
    warning('MPC: QP solver did not find optimal solution. Applying fallback.');
    u_opt = 0.0;
end
end