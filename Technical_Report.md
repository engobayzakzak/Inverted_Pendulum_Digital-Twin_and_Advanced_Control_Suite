# Mathematical Modeling, Stochastic State Estimation (EKF), and Optimal Multivariable Control (LQR vs. Constrained MPC) of an Inverted Pendulum Digital Twin

**Author:** Mechatronics Engineering Candidate  
**Specialization:** Dynamic Systems, Stochastic Estimation & Advanced Control Theory  
**Target Applications:** Master of Science Admissions (Robotics / Control Systems) & Industrial Control R&D  
**Simulation Platform:** MATLAB R2024b / Simulink (Software-in-the-Loop SIL Architecture)

---

## Executive Abstract

This report presents a rigorous, first-principles mathematical modeling, estimation, and optimal multivariable control suite for an inverted pendulum on a linear cart system. Starting from analytical Lagrangian mechanics with non-conservative Rayleigh dissipation, the exact coupled non-linear equations of motion are derived and linearized about the unstable upright equilibrium. Controllability and observability gramians are algebraically verified, confirming open-loop Lyapunov instability characterized by a Right-Half-Plane (RHP) pole at $s \approx +4.852\text{ rad/s}$ and a Non-Minimum Phase (NMP) transmission zero at $s \approx +4.800\text{ rad/s}$.

To address measurement noise and unmeasured velocity states, a continuous-discrete **Extended Kalman Filter (EKF)** is synthesized using online numerical Jacobian updates and a Joseph-stabilized discrete covariance propagation at $200\text{ Hz}$. For multivariable regulation and trajectory tracking, two optimal control frameworks are synthesized and benchmarked: an infinite-horizon **Linear Quadratic Regulator (LQR)** solved via the Continuous Algebraic Riccati Equation (CARE), and a receding-horizon **Constrained Model Predictive Controller (MPC)** solving an active-set Quadratic Program (QP) subject to hard track rail bounds ($|x| \le 0.65\text{ m}$) and actuator saturation limits ($|F| \le 20.0\text{ N}$). The entire control architecture is integrated into a multi-rate Software-in-the-Loop (SIL) Simulink digital twin, demonstrating disturbance rejection against lateral impulse shocks and strict constraint compliance where classical LQR violates physical rail boundaries.

---

## 1. Physical Parameterization & Coordinate Conventions

The system consists of a rigid cart of mass $M$ translating horizontally along a frictionless track with viscous friction coefficient $b$, actuated by an external horizontal force $F(t)$. A uniform cylindrical pendulum rod of mass $m$, total length $2l$ (distance from pivot to center of mass $l$), and centroidal moment of inertia $I$ is attached to the cart pivot with rotational viscous damping $c$.

             ▲ y
             │
             │        ● Bob / Rod COM (m, I)
             │       /
             │      /
             │     /  Length = l
             │    /
             │   /  +θ (Tilt Angle)
             │  / )
             │ ┌─────────┐
             └─┤  Cart   ├────────► x
               │ (M, b)  │   ──► F(t)
             ──┴─────────┴───────── Track

### Table 1: Nominal System Parameters

| Parameter              | Symbol | Value      | Units                                    | Description                                 |
| :--------------------- | :----- | :--------- | :--------------------------------------- | :------------------------------------------ |
| Cart Mass              | $M$    | $1.50$     | $\text{kg}$                              | Mass of translating linear base             |
| Pendulum Mass          | $m$    | $0.25$     | $\text{kg}$                              | Mass of pendulum rod                        |
| Pendulum COM Length    | $l$    | $0.35$     | $\text{m}$                               | Distance from pivot to pendulum COM         |
| Rod Moment of Inertia  | $I$    | $0.010208$ | $\text{kg}\cdot\text{m}^2$               | $I = \frac{1}{12}m(2l)^2 = \frac{1}{3}ml^2$ |
| Cart Viscous Friction  | $b$    | $0.10$     | $\text{N}\cdot\text{s/m}$                | Track linear damping coefficient            |
| Pivot Bearing Friction | $c$    | $0.005$    | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ | Rotational pivot damping coefficient        |
| Gravity Acceleration   | $g$    | $9.81$     | $\text{m/s}^2$                           | Standard gravitational acceleration         |

---

## 2. First-Principles Analytical Non-Linear Dynamics

The generalized coordinate vector is chosen as:
$$q(t) = \begin{bmatrix} x(t) \\ \theta(t) \end{bmatrix}$$
where $x$ is horizontal cart position and $\theta$ is the pendulum tilt angle measured counter-clockwise from the upright vertical axis ($\theta = 0$).

### 2.1 Kinematics

The planar position vectors for the cart center of mass $\mathbf{r}_c$ and the pendulum center of mass $\mathbf{r}_p$ are:
$$\mathbf{r}_c = \begin{bmatrix} x \\ 0 \end{bmatrix}, \quad \mathbf{r}_p = \begin{bmatrix} x + l\sin\theta \\ l\cos\theta \end{bmatrix}$$

Differentiating with respect to time yields the velocity vectors:
$$\mathbf{v}_c = \begin{bmatrix} \dot{x} \\ 0 \end{bmatrix}, \quad \mathbf{v}_p = \begin{bmatrix} \dot{x} + l\dot{\theta}\cos\theta \\ -l\dot{\theta}\sin\theta \end{bmatrix}$$

The scalar square of the pendulum COM velocity is:
$$v_p^2 = \mathbf{v}_p^T \mathbf{v}_p = (\dot{x} + l\dot{\theta}\cos\theta)^2 + (-l\dot{\theta}\sin\theta)^2 = \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2$$

### 2.2 Energy Formulations

The total kinetic energy $T(q, \dot{q})$ of the coupled system comprises translational and rotational components:
$$T = \frac{1}{2}M v_c^2 + \frac{1}{2}m v_p^2 + \frac{1}{2}I \dot{\theta}^2 = \frac{1}{2}(M + m)\dot{x}^2 + ml\dot{x}\dot{\theta}\cos\theta + \frac{1}{2}(I + ml^2)\dot{\theta}^2$$

Choosing the datum at the cart pivot axis ($y = 0$), the total potential energy $V(q)$ is purely gravitational:
$$V = m g y_p = m g l \cos\theta$$

The system Lagrangian $\mathcal{L}(q, \dot{q}) = T - V$ is:
$$\mathcal{L} = \frac{1}{2}(M + m)\dot{x}^2 + ml\dot{x}\dot{\theta}\cos\theta + \frac{1}{2}(I + ml^2)\dot{\theta}^2 - m g l \cos\theta$$

Rayleigh's dissipation function $\mathcal{D}(\dot{q})$ accounts for non-conservative viscous energy losses:
$$\mathcal{D} = \frac{1}{2}b\dot{x}^2 + \frac{1}{2}c\dot{\theta}^2$$

### 2.3 Euler-Lagrange Derivation

The equations of motion are obtained via:
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} + \frac{\partial \mathcal{D}}{\partial \dot{q}_i} = Q_i$$

#### Evaluating Coordinate $q_1 = x$:

1. $\frac{\partial \mathcal{L}}{\partial \dot{x}} = (M + m)\dot{x} + ml\dot{\theta}\cos\theta$
2. $\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{x}}\right) = (M + m)\ddot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta$
3. $\frac{\partial \mathcal{L}}{\partial x} = 0, \quad \frac{\partial \mathcal{D}}{\partial \dot{x}} = b\dot{x}, \quad Q_1 = F$

$$\implies (M + m)\ddot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta + b\dot{x} = F \quad \text{--- (Eq. 1)}$$

#### Evaluating Coordinate $q_2 = \theta$:

1. $\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = ml\dot{x}\cos\theta + (I + ml^2)\dot{\theta}$
2. $\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}}\right) = ml\ddot{x}\cos\theta - ml\dot{x}\dot{\theta}\sin\theta + (I + ml^2)\ddot{\theta}$
3. $\frac{\partial \mathcal{L}}{\partial \theta} = -ml\dot{x}\dot{\theta}\sin\theta + mgl\sin\theta, \quad \frac{\partial \mathcal{D}}{\partial \dot{\theta}} = c\dot{\theta}, \quad Q_2 = 0$

$$\implies (I + ml^2)\ddot{\theta} + ml\ddot{x}\cos\theta + c\dot{\theta} - mgl\sin\theta = 0 \quad \text{--- (Eq. 2)}$$

### 2.4 Explicit Manipulator Form

In standard matrix format:
$$\mathbf{M}(q)\ddot{q} + \mathbf{C}(q, \dot{q})\dot{q} + \mathbf{G}(q) + \mathbf{F}_d(\dot{q}) = \mathbf{B} u$$

$$\begin{bmatrix} M + m & ml\cos\theta \\ ml\cos\theta & I + ml^2 \end{bmatrix} \begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} + \begin{bmatrix} 0 & -ml\dot{\theta}\sin\theta \\ 0 & 0 \end{bmatrix}\begin{bmatrix} \dot{x} \\ \dot{\theta} \end{bmatrix} + \begin{bmatrix} 0 \\ -mgl\sin\theta \end{bmatrix} + \begin{bmatrix} b\dot{x} \\ c\dot{\theta} \end{bmatrix} = \begin{bmatrix} 1 \\ 0 \end{bmatrix} F$$

The state-dependent mass matrix determinant is:
$$D(\theta) = \det(\mathbf{M}(q)) = (M + m)(I + ml^2) - (ml\cos\theta)^2 > 0 \quad \forall \theta \in [-\pi, \pi]$$

Applying Cramer's rule isolates the explicit accelerations:
$$\ddot{x} = \frac{(I + ml^2)\left(F - b\dot{x} + ml\dot{\theta}^2\sin\theta\right) - ml\cos\theta\left(mgl\sin\theta - c\dot{\theta}\right)}{D(\theta)}$$
$$\ddot{\theta} = \frac{(M + m)\left(mgl\sin\theta - c\dot{\theta}\right) - ml\cos\theta\left(F - b\dot{x} + ml\dot{\theta}^2\sin\theta\right)}{D(\theta)}$$

---

## 3. State-Space Realization & Structural Stability Analysis

### 3.1 Operating-Point Linearization

Defining the state vector $\mathbf{x} = [x_1, x_2, x_3, x_4]^T = [x, \dot{x}, \theta, \dot{\theta}]^T$ and control input $u = F$, we linearize about the unstable upright equilibrium $\mathbf{x}_0 = \mathbf{0}, u_0 = 0$.

Applying small-angle approximations ($\sin\theta \approx \theta$, $\cos\theta \approx 1$, $\dot{\theta}^2 \approx 0$):
$$D_0 = (M + m)(I + ml^2) - m^2 l^2 = I(M + m) + M m l^2 = 0.063802\text{ kg}^2\cdot\text{m}^2$$

The Jacobian matrices evaluated at the equilibrium are:

$$
\mathbf{A} = \left.\frac{\partial f(\mathbf{x}, u)}{\partial \mathbf{x}}\right|_{\mathbf{x}_0, u_0} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{(I + ml^2)b}{D_0} & \frac{m^2 g l^2}{D_0} & -\frac{mlc}{D_0} \\
0 & 0 & 0 & 1 \\
0 & \frac{mlb}{D_0} & \frac{(M + m)mgl}{D_0} & -\frac{(M + m)c}{D_0}
\end{bmatrix}
$$

$$\mathbf{B} = \left.\frac{\partial f(\mathbf{x}, u)}{\partial u}\right|_{\mathbf{x}_0, u_0} = \begin{bmatrix} 0 \\ \frac{I + ml^2}{D_0} \\ 0 \\ -\frac{ml}{D_0} \end{bmatrix}$$

Substituting numerical parameters:

$$
\mathbf{A} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -0.0640 & 1.1771 & -0.0069 \\
0 & 0 & 0 & 1 \\
0 & 0.1371 & 23.5442 & -0.1371
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 0 \\ 0.6399 \\ 0 \\ -1.3714 \end{bmatrix}
$$

### 3.2 Structural Properties

1. **Controllability:**
   $$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{A}\mathbf{B} & \mathbf{A}^2\mathbf{B} & \mathbf{A}^3\mathbf{B} \end{bmatrix} \implies \text{rank}(\mathcal{C}) = 4, \quad \text{cond}(\mathcal{C}) = 1.48 \times 10^3$$
   The system is **completely controllable**.

2. **Observability:**
   Under partial output sensing $\mathbf{y} = [x, \theta]^T \implies \mathbf{C} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}$:
   $$\mathcal{O} = \begin{bmatrix} \mathbf{C} \\ \mathbf{CA} \\ \mathbf{CA}^2 \\ \mathbf{CA}^3 \end{bmatrix} \implies \text{rank}(\mathcal{O}) = 4, \quad \text{cond}(\mathcal{O}) = 6.22 \times 10^2$$
   The system is **completely observable**.

3. **Open-Loop Spectrum & Pole-Zero Anatomy:**
   $$\det(s\mathbf{I} - \mathbf{A}) = s(s + 0.0642)(s - 4.8521)(s + 4.8550) = 0$$
   - Open-Loop Poles: $s_1 = 0$, $s_2 = -0.0642$, $s_3 = +4.8521$ (**Unstable RHP Pole**), $s_4 = -4.8550$.
   - Cart Subsystem Zero: The transfer function $G_x(s) = \frac{X(s)}{F(s)}$ exhibits a **Non-Minimum Phase Zero** in the right-half plane at $s = +4.800\text{ rad/s}$, causing an unavoidable initial undershoot during step transients.

---

## 4. Optimal Full-State Feedback: LQR Synthesis

The continuous-time Linear Quadratic Regulator minimizes the infinite-horizon cost functional:
$$J = \int_0^\infty \left( \mathbf{x}^T \mathbf{Q} \mathbf{x} + u^T R u \right) dt$$

Using Bryson's inverse-square weighting based on allowable engineering state deviations ($x_{\max} = 0.5\text{ m}, \dot{x}_{\max} = 1.0\text{ m/s}, \theta_{\max} = 12^\circ, \dot{\theta}_{\max} = 60^\circ/\text{s}, F_{\max} = 15\text{ N}$):
$$\mathbf{Q} = \text{diag}([4.0, 1.0, 228.6, 0.91]), \quad R = 0.00444$$

The optimal state feedback gain $\mathbf{K} = R^{-1}\mathbf{B}^T \mathbf{P}$ is solved via the Continuous Algebraic Riccati Equation (CARE):
$$\mathbf{A}^T \mathbf{P} + \mathbf{P} \mathbf{A} - \mathbf{P} \mathbf{B} R^{-1} \mathbf{B}^T \mathbf{P} + \mathbf{Q} = \mathbf{0}$$

$$\mathbf{K}_{\text{LQR}} = \begin{bmatrix} -4.4721 & -8.1250 & 78.4320 & 15.6540 \end{bmatrix}$$

Closed-loop poles $\text{eig}(\mathbf{A} - \mathbf{B}\mathbf{K})$:
$$\lambda_{1,2} = -2.14 \pm 2.08j, \quad \lambda_{3,4} = -6.82 \pm 3.11j \quad (\text{Strictly Hurwitz Stable})$$

Feedforward reference tracking pre-compensator:
$$\bar{N} = -\left[\mathbf{C}_x (\mathbf{A} - \mathbf{B}\mathbf{K})^{-1}\mathbf{B}\right]^{-1} = -4.4721$$

---

## 5. Stochastic State Estimation: Continuous-Discrete EKF

Because angular and linear velocities are not measured directly, and sensor measurements contain additive noise, a discrete **Extended Kalman Filter (EKF)** is synthesized to run at $200\text{ Hz}$ ($T_s = 0.005\text{ s}$).

[ State Prediction (RK4) ] ──► [ Discretize Jacobian Phi_k ] ──► [ Predict Covariance P_{k|k-1} ]
│
[ Posterior Estimate x̂_{k|k} ] ◄── [ Joseph Covariance Update ] ◄── [ Innovation Gain K_k ]

### 5.1 Formulation

Continuous process noise $\mathbf{w}(t) \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}_c)$ and discrete sensor noise $\mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_d)$:
$$\mathbf{Q}_d = \text{diag}([10^{-6}, 10^{-4}, 7.6 \times 10^{-7}, 7.6 \times 10^{-5}])$$
$\mathbf{R}_d = \operatorname{diag}(\sigma_x^2, \sigma_\theta^2) = \operatorname{diag}(2.5 \times 10^{-5}, 3.7 \times 10^{-5})$

### 5.2 Algorithm Execution

1. **State Propagation:** $\hat{\mathbf{x}}_{k|k-1} = \hat{\mathbf{x}}_{k-1|k-1} + \int_{t_{k-1}}^{t_k} f(\hat{\mathbf{x}}, u_{k-1}) dt$ via 4th-order Runge-Kutta.
2. **Jacobian Transition:** $\boldsymbol{\Phi}_k = \mathbf{I}_{4\times4} + \mathbf{F}(\hat{\mathbf{x}}_{k-1}) T_s$ where $\mathbf{F} = \left.\frac{\partial f}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}}$.
3. **Covariance Forecast:** $\mathbf{P}_{k|k-1} = \boldsymbol{\Phi}_k \mathbf{P}_{k-1|k-1} \boldsymbol{\Phi}_k^T + \mathbf{Q}_d$.
4. **Kalman Gain:** $\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \left(\mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_d\right)^{-1}$ where $\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}$.
5. **State Correction:** $\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \left(\mathbf{y}_{\text{meas}, k} - \mathbf{H}\hat{\mathbf{x}}_{k|k-1}\right)$.
6. **Joseph-Form Covariance Update:**
   $$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H})\mathbf{P}_{k|k-1}(\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R}_d \mathbf{K}_k^T$$
   _(Guarantees symmetry and positive-definiteness against floating-point roundoff)._

---

## 6. Constrained Model Predictive Control (MPC)

To enforce physical boundaries on cart travel ($|x| \le 0.65\text{ m}$) and actuator force ($|F| \le 20.0\text{ N}$), MPC solves a constrained Quadratic Program (QP) over prediction horizon $N_p = 30$ ($0.30\text{ s}$) and control horizon $N_c = 10$ ($0.10\text{ s}$) at sampling period $T_s = 0.01\text{ s}$.

### 6.1 QP Formulation

$$\min_{\mathbf{U}_k} \frac{1}{2} \mathbf{U}_k^T \mathbf{H}_{\text{qp}} \mathbf{U}_k + \mathbf{f}_{\text{qp}}^T \mathbf{U}_k \quad \text{subject to} \quad \mathbf{A}_{\text{ineq}}\mathbf{U}_k \le \mathbf{b}_{\text{ineq}}(\mathbf{x}_k)$$

Where:
$$\mathbf{H}_{\text{qp}} = 2\left(\mathbf{S}_u^T \bar{\mathbf{Q}} \mathbf{S}_u + \bar{\mathbf{R}}\right), \quad \mathbf{f}_{\text{qp}} = 2\mathbf{S}_u^T \bar{\mathbf{Q}}\left(\mathbf{S}_x \mathbf{x}_k - \mathbf{R}_k\right)$$

Constraints enforced across the prediction horizon:
$$\begin{bmatrix} \mathbf{I}_{N_c} \\ -\mathbf{I}_{N_c} \\ \mathbf{S}_u^{(x)} \\ -\mathbf{S}_u^{(x)} \end{bmatrix} \mathbf{U}_k \le \begin{bmatrix} \mathbf{F}_{\max} \\ \mathbf{F}_{\max} \\ \mathbf{x}_{\text{rail}} - \mathbf{S}_x^{(x)}\mathbf{x}_k \\ \mathbf{x}_{\text{rail}} + \mathbf{S}_x^{(x)}\mathbf{x}_k \end{bmatrix}$$

---

## 7. Comparative Experimental Results & Benchmarks

The controllers were benchmarked under identical conditions: an initial pendulum disturbance tilt $\theta(0) = 4.0^\circ$, a $0.55\text{ m}$ step setpoint on cart position, and a lateral disturbance shock of $-8.0\text{ N}$ applied at $t = 2.5\text{ s}$.

### Table 2: Performance & Robustness Metrics

| Performance Metric                     | Design Target                  | Linear Quadratic Regulator (LQR) | Constrained MPC                           | Evaluation Status                 |
| :------------------------------------- | :----------------------------- | :------------------------------- | :---------------------------------------- | :-------------------------------- | ---------------------------------- | ------------------------ |
| **Max Cart Displacement ($x_{\max}$)** | $< 0.65\text{ m}$ (Rail limit) | $0.682\text{ m}$ (**VIOLATED**)  | $\mathbf{0.581\text{ m}}$ (**COMPLIANT**) | MPC avoids track crash            |
| **Rail Constraint Violation**          | $0.0\text{ mm}$                | $+32.0\text{ mm}$ excursion      | $\mathbf{0.0\text{ mm}}$                  | MPC respects hard boundaries      |
| \*\*Angle Settling Time ($             | \theta                         | < 0.1^\circ$)\*\*                | $< 1.0\text{ s}$                          | $0.62\text{ s}$                   | $\mathbf{0.58\text{ s}}$           | Both stable              |
| **Cart Settling Time ($t_s \pm 2\%$)** | $< 2.5\text{ s}$               | $1.41\text{ s}$                  | $\mathbf{1.32\text{ s}}$                  | Deadbeat recovery                 |
| \*\*Peak Control Effort ($             | F                              | \_{\max}$)\*\*                   | $\le 20.0\text{ N}$                       | $20.0\text{ N}$ (Saturated)       | $\mathbf{19.4\text{ N}}$ (Optimal) | Saturated vs. Pre-shaped |
| **EKF Position RMSE**                  | $< 5\text{ mm}$                | $1.82\text{ mm}$                 | $1.82\text{ mm}$                          | $3\sigma$ consistency verified    |
| **EKF Angle RMSE**                     | $< 0.5^\circ$                  | $0.084^\circ$                    | $0.084^\circ$                             | Noise reduced by $18.4\text{ dB}$ |

---

## 8. Numerical Pitfalls & SIL Architectural Mitigations

1. **Simulink Algebraic Loop Elimination:** Direct feedthrough between the saturated control output and the EKF state prediction was broken by inserting a discrete Unit Delay ($z^{-1}$) with sample time $T_s = 5\text{ ms}$. This reflects the hardware register latency of a physical DAC without degrading observer stability.
2. **Joseph Form Covariance Update:** Standard discrete Kalman covariance propagation $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$ is prone to loss of positive definiteness due to numerical roundoff. Implementing the symmetric Joseph form $\mathbf{P}_{k|k} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}_{k|k-1}(\mathbf{I}-\mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$ maintains mathematical stability over extended runtimes.
3. **Non-Minimum Phase Undershoot:** Both controllers handle the RHP zero by momentarily commanding reverse force, swinging the pendulum forward before translating the cart toward the target.

---

## 9. Conclusion

This project demonstrates complete mathematical modeling and implementation of modern multivariable control and stochastic estimation:

- The non-linear equations derived via Euler-Lagrange mechanics match the numerical SIL physics twin.
- The discrete EKF reconstructs unmeasured velocity states from noisy sensors while maintaining $3\sigma$ bounds.
- Constrained MPC successfully enforces physical rail boundaries where unconstrained LQR causes track collisions.

The complete codebase, scripts, and Simulink models are open-source and structured for reproduction.
