# Mathematical Foundations Roadmap for Humanoid Robotics

**Goal**: Build the mathematical foundation needed to transition from autonomous vehicles to humanoid robotics, following the Learning Roadmap.

**Total Duration**: ~6 months (adjustable based on background)

---

## Overview: The Complete Sequence

```
Phase 1: Linear Algebra (4 weeks)
    ↓
Phase 2: Classical Mechanics (3 weeks)
    ↓
Phase 3: Underactuated Robotics (6 weeks)
    ↓
Phase 4: Probability & State Estimation (4 weeks)
    ↓
Phase 5: 3D Geometry (3 weeks)
    ↓
→ Ready for Tier 1 Robotics Skills (State Estimation, MPC, RL)
```

---

## Master Resource List

| Phase | Primary Resource | Supplementary | URL |
|-------|------------------|---------------|-----|
| 1 | Boyd's *Applied Linear Algebra* | 3Blue1Brown | [Book](https://web.stanford.edu/~boyd/vmls/) |
| 2 | Susskind's *Classical Mechanics* | van Biezen (quick option) | [YouTube](https://www.youtube.com/playlist?list=PL47F408D36D4CF129) |
| 3 | Tedrake's *Underactuated Robotics* | Brunton's Control Bootcamp | [Website](https://underactuated.mit.edu) |
| 4 | Thrun's *Probabilistic Robotics* | MIT 6.041 | [Book](https://mitpress.mit.edu/9780262201629/) |
| 5 | Lynch's *Modern Robotics* | - | [Book](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) |

---

# Phase 1: Linear Algebra (4 Weeks)

**Resources:**
- *Introduction to Applied Linear Algebra* (Boyd & Vandenberghe) - [Free PDF](https://web.stanford.edu/~boyd/vmls/)
- 3Blue1Brown "Essence of Linear Algebra" - [YouTube](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab)

**Time Commitment**: ~2 hours/day

---

## Week 1: Vectors & Matrices (Foundations)

### Day 1-2: Geometric Intuition First
- [ ] Watch 3Blue1Brown videos 1-5 (~1.5 hrs)
  - [ ] Video 1: Vectors, what even are they?
  - [ ] Video 2: Linear combinations, span, and basis vectors
  - [ ] Video 3: Linear transformations and matrices
  - [ ] Video 4: Matrix multiplication as composition
  - [ ] Video 5: Three-dimensional linear transformations
- [ ] Read Boyd Ch. 1 (Vectors)
- [ ] Read Boyd Ch. 2 (Linear functions)

### Day 3-4: Matrix Operations
- [ ] Read Boyd Ch. 6 (Matrices)
- [ ] Read Boyd Ch. 7 (Matrix examples)
- [ ] **Implement from scratch**:
  - [ ] Matrix-vector multiplication
  - [ ] Matrix-matrix multiplication
  - [ ] Transpose operation

### Day 5-7: Linear Independence & Systems
- [ ] Read Boyd Ch. 5 (Linear independence)
- [ ] Read Boyd Ch. 8 (Linear equations)
- [ ] Watch 3Blue1Brown videos 6-7
  - [ ] Video 6: The determinant
  - [ ] Video 7: Inverse matrices, column space, and null space
- [ ] **Implement**: Gaussian elimination

### Week 1 Checkpoint
> **Can you explain what Ax = b means geometrically?**
>
> Notes: _____________________________________________

---

## Week 2: Core Decompositions

### Day 8-9: Orthogonality
- [ ] Read Boyd Ch. 3 (Norm and distance)
- [ ] Read Boyd Ch. 5.3-5.4 (Orthonormal vectors, Gram-Schmidt)
- [ ] Watch 3Blue1Brown video on dot products (Video 9)
- [ ] Watch 3Blue1Brown videos 10-11 (Cross products) - **Boyd gap filler**
- [ ] **Implement**: Gram-Schmidt orthogonalization

### Day 10-11: QR Decomposition
- [ ] Read Boyd Ch. 10 (Matrix multiplication as composition)
- [ ] Understand QR = orthogonal × upper triangular
- [ ] **Implement**: QR decomposition using Gram-Schmidt
- [ ] Use QR to solve least squares

### Day 12-14: Eigenvalues & Eigenvectors
- [ ] Watch 3Blue1Brown videos 13-14
  - [ ] Video 13: Eigenvectors and eigenvalues
  - [ ] Video 14: A quick trick for computing eigenvalues
- [ ] Watch Brunton Control Bootcamp videos 4-6 (eigenvalues for stability)
- [ ] **Implement**: Power iteration for dominant eigenvalue
- [ ] Understand: why eigenvalues matter for stability analysis

### Week 2 Checkpoint
> **Given a matrix, can you find its eigenvalues and explain what they mean?**
>
> Notes: _____________________________________________

---

## Week 3: Least Squares & SVD

### Day 15-17: Least Squares (Boyd's Strength)
- [ ] Read Boyd Ch. 12 (Least squares)
- [ ] Read Boyd Ch. 13 (Least squares data fitting)
- [ ] Read Boyd Ch. 14 (Least squares classification)
- [ ] **Implement**: Linear regression from scratch
- [ ] **Implement**: Polynomial fitting

### Day 18-21: Singular Value Decomposition
- [ ] Watch 3Blue1Brown video on change of basis (Video 12)
- [ ] Watch Brunton videos 32-35 (SVD for control)
- [ ] Understand: A = UΣV^T geometrically (rotate, scale, rotate)
- [ ] **Implement**: Use `numpy.linalg.svd`, understand outputs
- [ ] **Apply**: Low-rank approximation demo
- [ ] **Apply**: Image compression using SVD

### Week 3 Checkpoint
> **Can you explain SVD geometrically and use it for dimensionality reduction?**
>
> Notes: _____________________________________________

---

## Week 4: Applications & Consolidation

### Day 22-23: Positive Definite Matrices
- [ ] Understand: x^T A x > 0 for all x ≠ 0
- [ ] Connection to: covariance matrices, optimization (Hessians)
- [ ] Cholesky decomposition: A = LL^T

### Day 24-25: Matrix Calculus Basics
- [ ] Gradients of matrix expressions
- [ ] Jacobians (input-output relationships)
- [ ] Chain rule for matrices

### Day 26-28: Robotics Applications Mini-Projects
- [ ] **Mini-project 1**: Least squares for line/curve fitting on sensor data
- [ ] **Mini-project 2**: Use SVD for a simple state estimation problem
- [ ] **Mini-project 3**: Eigenvalue analysis of a simple dynamical system

### Phase 1 Final Checkpoint
- [ ] Can you solve Ax = b multiple ways and know when to use which?
- [ ] Can you compute and interpret SVD?
- [ ] Can you set up a least squares problem from a real scenario?

---

# Phase 2: Classical Mechanics (3 Weeks)

**Purpose**: Build foundation for understanding robot dynamics (Lagrangian/Hamiltonian mechanics)

**Resources:**
- Leonard Susskind's *Classical Mechanics: The Theoretical Minimum* - [YouTube](https://www.youtube.com/playlist?list=PL47F408D36D4CF129)
- **Quick alternative**: Michel van Biezen Lagrangian playlist (if short on time)

**Time Commitment**: ~1.5 hours/day

---

## Week 5: Foundations of Analytical Mechanics

### Day 29-31: State Space and Newton Review
- [ ] Susskind Lecture 1: The nature of classical physics
- [ ] Susskind Lecture 2: Motion and state space
- [ ] Review: Newton's laws, F = ma in multiple dimensions
- [ ] Understand: Phase space (position + velocity)

### Day 32-35: Principle of Least Action
- [ ] Susskind Lecture 3: Principle of least action
- [ ] Susskind Lecture 4: Symmetry and conservation laws
- [ ] Understand: Action = ∫L dt, and why nature minimizes it
- [ ] **Work through**: Derive equations of motion for free particle

### Week 5 Checkpoint
> **Can you explain what "action" means and why it's minimized?**
>
> Notes: _____________________________________________

---

## Week 6: Lagrangian Mechanics

### Day 36-38: Lagrangian Formulation
- [ ] Susskind Lecture 5: Lagrangian mechanics
- [ ] Susskind Lecture 6: Euler-Lagrange equations
- [ ] Master the equation: d/dt(∂L/∂q̇) - ∂L/∂q = 0
- [ ] **Derive by hand**: Simple pendulum equations of motion

### Day 39-42: Practice Problems
- [ ] **Derive**: Double pendulum (this is hard - take your time)
- [ ] **Derive**: Mass on a spring
- [ ] **Derive**: Particle on an inclined plane
- [ ] Compare: Lagrangian vs Newtonian approach (which is easier?)

### Week 6 Checkpoint
> **Can you derive equations of motion for a pendulum using Lagrangian mechanics?**
>
> Notes: _____________________________________________

---

## Week 7: Hamiltonian Mechanics & Bridge to Robotics

### Day 43-45: Hamiltonian Formulation
- [ ] Susskind Lecture 7: Hamiltonian mechanics
- [ ] Susskind Lecture 8: Poisson brackets (light coverage OK)
- [ ] Understand: H = T + V (total energy), Legendre transform
- [ ] Connection: Hamiltonian → conservation of energy

### Day 46-49: Bridge to Robotics
- [ ] Read: Tedrake Ch. 2.1-2.2 (preview of robot dynamics)
- [ ] Understand: How Lagrangian becomes M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
- [ ] **Implement**: Simulate simple pendulum using derived equations

### Phase 2 Final Checkpoint
- [ ] Can you explain Lagrangian vs Newtonian mechanics?
- [ ] Can you derive equations of motion for a 2-DOF system?
- [ ] Do you understand why robotics uses Lagrangian formulation?

---

# Phase 3: Underactuated Robotics (6 Weeks)

**Purpose**: Learn robotics dynamics, trajectory optimization, and control

**Resources:**
- Russ Tedrake's *Underactuated Robotics* - [Website](https://underactuated.mit.edu)
- Steve Brunton's *Control Bootcamp* (supplement) - [YouTube](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
- Drake toolbox for Python exercises

**Time Commitment**: ~2 hours/day

---

## Week 8-9: Dynamics Foundations

### Nonlinear Dynamics
- [ ] Tedrake Ch. 1: Introduction (fully actuated vs underactuated)
- [ ] Tedrake Ch. 2: Pendulum dynamics (builds on your Lagrangian knowledge)
- [ ] Brunton videos 1-3: Overview of dynamical systems
- [ ] **Run**: Drake notebook examples for pendulum

### Stability Analysis
- [ ] Tedrake Ch. 2 (stability sections)
- [ ] Brunton videos 4-8: Stability and eigenvalues
- [ ] Understand: Lyapunov stability, linearization around equilibria
- [ ] **Implement**: Analyze stability of inverted pendulum

### Weeks 8-9 Checkpoint
> **Can you determine if a system is stable by analyzing its linearization?**
>
> Notes: _____________________________________________

---

## Week 10-11: Trajectory Optimization

### Dynamic Programming
- [ ] Tedrake Ch. 3: Acrobot and cart-pole
- [ ] Tedrake Ch. 7: Dynamic programming basics
- [ ] Understand: Value functions, Bellman equation
- [ ] **Run**: Value iteration examples in Drake

### Direct Methods
- [ ] Tedrake Ch. 10: Trajectory optimization
- [ ] Understand: Shooting methods vs collocation
- [ ] Brunton videos on optimization (if available)
- [ ] **Implement**: Simple trajectory optimization problem

### Weeks 10-11 Checkpoint
> **Can you set up a trajectory optimization problem and explain the constraints?**
>
> Notes: _____________________________________________

---

## Week 12-13: Linear Optimal Control

### LQR (Linear Quadratic Regulator)
- [ ] Tedrake Ch. 8: LQR
- [ ] Brunton videos 15-25: Full LQR coverage
- [ ] Understand: Q and R matrices, cost function design
- [ ] **Implement**: LQR controller for cart-pole balancing

### MPC Foundations
- [ ] Tedrake Ch. 10 (MPC sections)
- [ ] Understand: Receding horizon, constraints
- [ ] **Implement**: Simple MPC for a linear system

### Weeks 12-13 Checkpoint
> **Can you design an LQR controller and tune the Q/R matrices?**
>
> Notes: _____________________________________________

### Phase 3 Final Checkpoint
- [ ] Can you analyze stability of a nonlinear system?
- [ ] Can you set up and solve a trajectory optimization problem?
- [ ] Can you implement LQR and basic MPC?

---

# Phase 4: Probability & State Estimation (4 Weeks)

**Purpose**: Foundation for Kalman filters, sensor fusion, SLAM

**Resources:**
- *Probabilistic Robotics* by Thrun, Burgard, Fox (primary)
- MIT 6.041 *Probabilistic Systems Analysis* (supplement) - [MIT OCW](https://ocw.mit.edu/courses/6-041-probabilistic-systems-analysis-and-applied-probability-fall-2010/)
- FilterPy library for implementations

**Time Commitment**: ~2 hours/day

---

## Week 14: Probability Foundations

### Day 1-3: Probability Basics
- [ ] Thrun Ch. 2.1-2.2: Probability review
- [ ] MIT 6.041 Lectures 1-3 (if needed)
- [ ] Master: Bayes' theorem, conditional probability
- [ ] Understand: Prior, likelihood, posterior

### Day 4-7: Gaussian Distributions
- [ ] Thrun Ch. 2.3-2.4: Gaussian distributions
- [ ] Understand: Multivariate Gaussians, covariance matrices
- [ ] Connection: Why Gaussians are everywhere in robotics
- [ ] **Implement**: Sample from multivariate Gaussian, visualize covariance

### Week 14 Checkpoint
> **Can you apply Bayes' theorem and work with multivariate Gaussians?**
>
> Notes: _____________________________________________

---

## Week 15-16: Kalman Filtering

### Linear Kalman Filter
- [ ] Thrun Ch. 3.1-3.2: Kalman filter derivation
- [ ] Understand: Predict step, update step
- [ ] Understand: Why covariance propagation matters
- [ ] **Implement**: 1D Kalman filter from scratch

### Extended Kalman Filter
- [ ] Thrun Ch. 3.3: EKF for nonlinear systems
- [ ] Understand: Jacobian linearization
- [ ] **Implement**: EKF for a simple nonlinear system
- [ ] **Apply**: GPS + IMU fusion (connect to your AV platform!)

### Weeks 15-16 Checkpoint
> **Can you implement an EKF and explain when it fails?**
>
> Notes: _____________________________________________

---

## Week 17: Advanced Topics

### Particle Filters
- [ ] Thrun Ch. 4: Particle filters
- [ ] Understand: When to use particle filters vs Kalman filters
- [ ] **Implement**: Simple particle filter for localization

### Factor Graphs (Overview)
- [ ] Thrun Ch. 11 (skim) or Barfoot Ch. 4
- [ ] Understand: Batch vs recursive estimation
- [ ] Know: GTSAM exists for factor graph optimization

### Phase 4 Final Checkpoint
- [ ] Can you implement a Kalman filter from scratch?
- [ ] Can you fuse multiple sensors using EKF?
- [ ] Do you understand the probabilistic interpretation of state estimation?

---

# Phase 5: 3D Geometry & Transforms (3 Weeks)

**Purpose**: Rotations, transformations, and spatial reasoning for robotics

**Resources:**
- *Modern Robotics* by Lynch & Park, Chapters 3-4 - [Free PDF](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- *A Mathematical Introduction to Robotic Manipulation* by Murray, Li, Sastry (deeper)

**Time Commitment**: ~1.5 hours/day

---

## Week 18: Rotations

### Rotation Representations
- [ ] Lynch Ch. 3.1-3.2: Rotation matrices, SO(3)
- [ ] Understand: Why rotation matrices must be orthogonal with det = 1
- [ ] Learn: Euler angles and gimbal lock problem
- [ ] **Implement**: Convert between rotation matrix and Euler angles

### Quaternions
- [ ] Supplementary material on quaternions
- [ ] Understand: Why quaternions avoid gimbal lock
- [ ] Understand: Quaternion multiplication = rotation composition
- [ ] **Implement**: Quaternion to rotation matrix conversion

### Week 18 Checkpoint
> **Can you explain gimbal lock and why quaternions solve it?**
>
> Notes: _____________________________________________

---

## Week 19: Rigid Body Transforms

### Homogeneous Coordinates
- [ ] Lynch Ch. 3.3: Homogeneous transformation matrices
- [ ] Understand: SE(3) = rotation + translation
- [ ] Understand: Transform composition and inverse
- [ ] **Implement**: Chain of transforms for a robot arm

### Exponential Coordinates
- [ ] Lynch Ch. 3.4: Exponential coordinates (so(3), se(3))
- [ ] Understand: Rodrigues' formula
- [ ] Connection: Lie groups and Lie algebras (conceptual)

### Week 19 Checkpoint
> **Can you compose multiple transforms and find the inverse?**
>
> Notes: _____________________________________________

---

## Week 20: Kinematics

### Forward Kinematics
- [ ] Lynch Ch. 4: Product of exponentials formula
- [ ] Understand: How joint angles map to end-effector pose
- [ ] **Implement**: FK for a 3-DOF arm

### Jacobians and Velocity
- [ ] Lynch Ch. 5: Velocity kinematics
- [ ] Understand: Jacobian maps joint velocities to end-effector velocity
- [ ] Understand: Singularities (det(J) = 0)
- [ ] **Implement**: Compute Jacobian for simple arm

### Phase 5 Final Checkpoint
- [ ] Can you work with rotation matrices, Euler angles, and quaternions?
- [ ] Can you compose and invert SE(3) transforms?
- [ ] Can you compute forward kinematics and identify singularities?

---

# Summary: What You've Built

After completing all 5 phases (~6 months), you have:

| Skill | Enables |
|-------|---------|
| Linear algebra fluency | Everything else |
| Lagrangian mechanics | Understanding robot dynamics |
| Trajectory optimization & LQR/MPC | Tier 1: MPC skill |
| Kalman filtering & sensor fusion | Tier 1: State Estimation skill |
| 3D geometry & kinematics | Tier 2: Vision, SLAM foundations |

**You're now ready for:**
- Tier 1: State Estimation (deeper with GTSAM)
- Tier 1: Trajectory Optimization (deeper with CasADi/ACADOS)
- Tier 1: Reinforcement Learning (Isaac Gym)

---

## Implementation Code Directory

```
exercises/
├── phase1_linear_algebra/
│   ├── week1/
│   ├── week2/
│   ├── week3/
│   └── week4/
├── phase2_classical_mechanics/
│   ├── pendulum_lagrangian.py
│   ├── double_pendulum.py
│   └── simulate_dynamics.py
├── phase3_underactuated/
│   ├── stability_analysis.py
│   ├── lqr_cartpole.py
│   └── trajectory_opt.py
├── phase4_probability/
│   ├── kalman_filter.py
│   ├── ekf_gps_imu.py
│   └── particle_filter.py
└── phase5_geometry/
    ├── rotations.py
    ├── transforms.py
    └── forward_kinematics.py
```

---

## Progress Log

| Date | Phase | Topics Covered | Time Spent | Notes |
|------|-------|----------------|------------|-------|
| | | | | |
| | | | | |
| | | | | |

---

## Quick Reference: Key Equations

**Linear Algebra:**
- Least squares: x = (A^T A)^(-1) A^T b
- SVD: A = UΣV^T
- Eigenvalue: Av = λv

**Lagrangian Mechanics:**
- Lagrangian: L = T - V
- Euler-Lagrange: d/dt(∂L/∂q̇) - ∂L/∂q = τ
- Robot form: M(q)q̈ + C(q,q̇)q̇ + g(q) = τ

**Kalman Filter:**
- Predict: x̂ = Ax + Bu, P = APA^T + Q
- Update: K = PH^T(HPH^T + R)^(-1), x̂ = x̂ + K(z - Hx̂)

**Rotations:**
- SO(3): R^T R = I, det(R) = 1
- Quaternion: q = [w, x, y, z], ||q|| = 1
