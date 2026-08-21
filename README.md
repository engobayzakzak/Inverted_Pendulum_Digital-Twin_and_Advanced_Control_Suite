# Inverted Pendulum Digital Twin & Advanced Control Suite

> **Software-in-the-Loop (SIL) Mathematical Simulation, Extended Kalman Filter (EKF) State Observer, and LQR vs. Constrained Model Predictive Control (MPC) in MATLAB & Simulink.**

[![MATLAB](https://img.shields.io/badge/MATLAB-R2022a-orange.svg)](https://www.mathworks.com/)
[![Simulink](https://img.shields.io/badge/Simulink-SIL%20Architecture-blue.svg)](https://www.mathworks.com/products/simulink.html)

---

## Overview

This repository contains a first-principles control engineering project implementing an **Inverted Pendulum Digital Twin**. It includes exact analytical Euler-Lagrange non-linear dynamics, continuous-time Jacobian linearization, structural controllability/observability analysis, stochastic state estimation via a discrete **Extended Kalman Filter (EKF)**, and multivariable control using **LQR** and **Constrained MPC**.

The system is tested within an algebraic-loop-free **Simulink Software-in-the-Loop (SIL)** environment under sensor noise and lateral disturbance shocks.

[ Reference Target r(t) ] ──► [ Optimal Controller (LQR / MPC) ] ──► [ Saturation [-25N, +25N] ]
▲ │
│ Full Estimated State x̂ ▼
[ Noisy Sensors (Encoder/IMU) ] ──► [ Discrete EKF Observer ] ◄── [ Non-Linear Physics Twin ]

---

## Key Technical Features

- **Non-Linear Dynamics:** Derived via Euler-Lagrange mechanics with Rayleigh dissipation; integrated using 4th-order Runge-Kutta (RK4).
- **Open-Loop Instability Analysis:** Verified open-loop RHP unstable pole ($s = +4.852\text{ rad/s}$) and Non-Minimum Phase zero ($s = +4.800\text{ rad/s}$).
- **Continuous Algebraic Riccati Solver (CARE):** Infinite-horizon LQR gain synthesis based on Bryson's rule.
- **Continuous-Discrete EKF Observer ($200\text{ Hz}$):** Online Jacobian updates and Joseph-stabilized covariance propagation with $3\sigma$ error bounding.
- **Constrained MPC ($100\text{ Hz}$):** Receding-horizon active-set Quadratic Program (QP) enforcing physical rail boundaries ($|x| \le 0.65\text{ m}$) and actuator force limits ($|F| \le 20\text{ N}$).
- **Simulink Multi-Rate SIL Harness:** Continuous plant integration paired with discrete digital control layers, free of algebraic loops.

---

## Quantitative Benchmarks (LQR vs. Constrained MPC)

| Metric | Target Specification | Unconstrained LQR | Constrained MPC | Result |
| :--- | :--- | :--- | :--- | :--- |
| **Rail Constraint ($\lvert x \rvert \le 0.65\text{ m}$)** | Zero Violation | $0.682\text{ m}$ (**VIOLATED**) | $\mathbf{0.581\text{ m}}$ (**COMPLIANT**) | **MPC avoids rail collision** |
| **Angle Settling Time ($\lvert \theta \rvert < 0.1^\circ$)** | $< 1.0\text{ s}$ | $0.62\text{ s}$ | $\mathbf{0.58\text{ s}}$ | **PASSED** |
| **Cart Settling Time ($t_s \pm 2\%$)** | $< 2.5\text{ s}$ | $1.41\text{ s}$ | $\mathbf{1.32\text{ s}}$ | **PASSED** |
| **EKF Position Estimation RMSE** | $< 5.0\text{ mm}$ | $1.82\text{ mm}$ | $1.82\text{ mm}$ | **PASSED** |
| **EKF Angle Estimation RMSE** | $< 0.50^\circ$ | $0.084^\circ$ | $0.084^\circ$ | **PASSED** |
---

## Repository Structure

```text
├── README.md                          # Executive project overview and benchmark table
├── Technical_Report.md                # Comprehensive mathematical whitepaper
├── system_params.m                    # Plant parameters, Jacobian matrices, CARE solver
├── nonlinear_dynamics.m               # Vectorized exact non-linear ODEs
├── ekf_observer.m                     # Continuous-discrete EKF function
├── mpc_controller.m                   # Constrained QP receding-horizon solver
├── run_sil_simulation.m               # Standalone non-linear LQR simulation script
├── run_ekf_lqr_simulation.m           # Closed-loop non-linear plant + EKF + LQR runner
├── run_mpc_vs_lqr_benchmark.m         # Head-to-head MPC vs. LQR benchmark suite
├── build_simulink_model.m             # Programmatic Simulink model builder
├── inverted_pendulum_digital_twin.slx  # Multi-rate Simulink SIL harness
└── run_simulink_verification.m        # Automated test execution and plotting script
```

---

## Quick Start (Reproducibility)

### Requirements

- MATLAB R2022a or later
- Simulink
- Control System Toolbox
- Optimization Toolbox (for `quadprog` in MPC)

### Execution Steps

1. **Initialize Parameters & Calculate Matrices:**
   ```matlab
   system_params
   Run the EKF + LQR Closed-Loop Simulation:
   ```

Matlab
run_ekf_lqr_simulation
Execute the MPC vs. LQR Rail-Constraint Benchmark:

Matlab
run_mpc_vs_lqr_benchmark
Build & Execute the Simulink Digital Twin:

Matlab
build_simulink_model
run_simulink_verification
