# Strategic Learning Roadmap: From Autonomous Vehicles to Humanoid Robotics

## Your Current Position

Based on analyzing your UTM Navigator codebase, you've built production-grade autonomous vehicle software with:

**Strong foundations in:**
- Vehicle kinematics & coordinate systems (WGS84/UTM/Vehicle frame transforms)
- Path tracking (Pure Pursuit with adaptive lookahead)
- Reactive local planning (DWA with Ackermann constraints)
- Grid-based mapping (log-odds Bayesian updates, Bresenham raycasting)
- Real hardware integration (Xsens RTK-GPS, Velodyne LIDAR, Teensy CAN)
- Production software practices (threading, config management, modular architecture)

**Key gaps identified:**
- No state estimation (Kalman filters, sensor fusion)
- No trajectory optimization (MPC, LQR)
- No reinforcement learning or sim2real
- Vision/deep learning exists but disconnected from control loop
- No dexterous manipulation experience

---

## The LLM Era Reality Check

You're right that the landscape has changed. Here's what LLMs **can** and **cannot** do for robotics engineers in 2026:

### LLMs CAN handle (delegate freely):
- Boilerplate code generation
- API integrations and driver code
- Documentation and technical writing
- Debugging known patterns
- Code refactoring and cleanup
- Basic algorithm implementations (PID, A*, etc.)

### LLMs CANNOT replace (must master deeply):
- **Physical intuition** - understanding why a robot falls, drifts, or fails
- **System debugging** - tracing issues across sensors → perception → planning → control → hardware
- **Dynamics modeling** - choosing the right simplifications for real-time control
- **Reward engineering** - designing cost functions that produce desired emergent behavior
- **Sim2real transfer** - understanding what physics matter and what can be randomized
- **Safety reasoning** - knowing failure modes and designing fallbacks

**Key insight**: LLMs are tools, not replacements. The engineers who thrive will be those who can specify problems precisely, validate solutions physically, and debug across the full stack.

---

## The Learning Roadmap

### TIER 1: FOUNDATIONAL (Deep Study Required)

These are the "non-negotiable" skills that everything else builds on. You cannot shortcut these.

---

#### 1. State Estimation & Sensor Fusion
**Depth: DEEP (3-4 months focused study)**

**Why it matters:** Every legged robot needs to know where it is and how fast it's moving. Your codebase has zero state estimation - you're reading raw GPS. Real systems fuse GPS, IMU, wheel encoders, and visual odometry.

**What to learn:**
- Kalman Filter fundamentals (linear systems)
- Extended Kalman Filter (EKF) for nonlinear systems
- Unscented Kalman Filter (UKF) - better for highly nonlinear
- Factor graphs and batch optimization (GTSAM)
- IMU preintegration

**Practical project:** Add an EKF to your AV platform that fuses GPS + IMU. You'll immediately see smoother position estimates and can detect GPS dropouts.

**Resources:**
- Book: "Probabilistic Robotics" by Thrun, Burgard, Fox (the bible)
- Book: "State Estimation for Robotics" by Tim Barfoot (more modern)
- Course: Coursera "State Estimation and Localization for Self-Driving Cars"
- Library: FilterPy (Python), GTSAM (C++/Python)

**Connection to humanoids:** Legged robots have no wheel encoders. They must fuse IMU + kinematics + contact detection. Same math, harder problem.

---

#### 2. Trajectory Optimization & MPC
**Depth: DEEP (3-4 months focused study)**

**Why it matters:** Your Pure Pursuit + PID is reactive - it sees the path and follows. MPC is predictive - it plans trajectories that respect constraints (motor limits, stability, obstacles) over a horizon.

**What to learn:**
- Linear MPC formulation
- Nonlinear MPC (harder but necessary for complex dynamics)
- Quadratic Programs (QP) and solvers (OSQP, ECOS)
- iLQR (iterative Linear Quadratic Regulator)
- Whole-body control for humanoids

**Practical project:** Replace your Pure Pursuit with MPC for path tracking. You'll see better performance in sharp turns and can add speed-dependent constraints.

**Resources:**
- Course: MIT 6.832 "Underactuated Robotics" (Russ Tedrake) - FREE
- Book: "Predictive Control for Linear and Hybrid Systems" (Borrelli)
- Library: CasADi, ACADOS, Drake, Crocoddyl
- Tutorial: Machines in Motion Lab MPC resources

**Connection to humanoids:** MPC is how Atlas, Digit, and Optimus maintain balance. They're solving optimization problems at 100Hz+ to generate joint torques.

---

#### 3. Reinforcement Learning & Sim2Real
**Depth: DEEP (4-6 months)**

**Why it matters:** This is how legged locomotion is solved now. Every humanoid company (Figure, Agility, Tesla, Boston Dynamics) uses RL trained in simulation, transferred to real hardware.

**What to learn:**
- RL fundamentals (MDPs, policy gradients, value functions)
- PPO (Proximal Policy Optimization) - the workhorse algorithm
- Domain randomization for sim2real
- Reward shaping and curriculum learning
- Sim2sim validation

**Practical project:** Train a simulated quadruped (Unitree Go1) to walk using Isaac Gym or MuJoCo. This is the on-ramp to humanoid RL.

**Resources:**
- Course: UC Berkeley CS 285 "Deep RL" (Sergey Levine) - FREE
- Framework: Isaac Gym / Isaac Lab (NVIDIA) or MuJoCo + Brax
- Code: Humanoid-Gym (https://github.com/roboterax/humanoid-gym)
- Code: Unitree RL Gym (https://github.com/unitreerobotics/unitree_rl_gym)
- Paper: "Learning Agile Locomotion" (ETH Zurich)

**Connection to humanoids:** This is THE skill for humanoid locomotion. Companies literally hire based on Isaac Gym experience.

---

### TIER 2: HIGH-LEVERAGE (Solid Understanding Required)

These amplify your Tier 1 skills and are directly applicable to humanoid work.

---

#### 4. Rigid Body Dynamics & Contact Mechanics
**Depth: MEDIUM-DEEP (2-3 months)**

**Why it matters:** Humanoids are complex multi-body systems. You need to understand Lagrangian/Hamiltonian mechanics, constraint forces, and how contact works.

**What to learn:**
- Spatial vector algebra (Featherstone notation)
- Forward/inverse dynamics
- Contact modeling (compliant vs rigid)
- Friction cones and contact wrenches

**Resources:**
- Book: "Rigid Body Dynamics Algorithms" by Roy Featherstone
- Library: Pinocchio (fast dynamics library)
- MIT 6.832 covers this well

**Connection to your work:** Your bicycle model is a start, but humanoids have 30+ DOF with closed kinematic chains.

---

#### 5. Computer Vision for Robotics
**Depth: MEDIUM (2-3 months)**

**Why it matters:** You have YOLOPv2 in your codebase but it's not integrated. Humanoids need vision for manipulation, navigation, and human interaction.

**What to learn:**
- Visual odometry / Visual SLAM
- Depth estimation and 3D reconstruction
- Object detection and pose estimation
- Foundation models (CLIP, SAM, DINO)

**Resources:**
- Course: University of Bonn "Photogrammetry" (Cyrill Stachniss)
- Library: OpenCV, ORB-SLAM3, DROID-SLAM
- Modern: Use foundation models via API when possible

**Connection to humanoids:** Figure AI uses vision extensively for manipulation. Not as critical as locomotion skills but important.

---

#### 6. Real-Time Systems
**Depth: MEDIUM (1-2 months)**

**Why it matters:** Your Python code runs at "soft real-time" (best effort). Production humanoids need hard guarantees - 1kHz control loops that never miss a deadline.

**What to learn:**
- PREEMPT_RT Linux
- Real-time priorities and scheduling
- Lock-free programming patterns
- ROS2 real-time considerations

**Resources:**
- Book: "Real-Time Systems" by Jane Liu
- Practical: Run your control loop at 1kHz on PREEMPT_RT Linux

**Connection to your work:** Your 20Hz control loop is fine for slow AV navigation. Humanoid balance requires 500Hz-1kHz.

---

### TIER 3: NICE-TO-HAVE (Learn As Needed)

These are valuable but can be learned on-demand or delegated to LLMs for assistance.

---

#### 7. ROS2 & Robot Middleware
**Depth: LIGHT (learn when needed)**

**Why:** Industry standard, but you can learn it when you join a team that uses it. Your current modular architecture is actually cleaner for research.

#### 8. CAD & Mechanical Design
**Depth: LIGHT**

**Why:** As a software/controls person, you'll collaborate with mechanical engineers. Basic Fusion360/SolidWorks literacy helps communication.

#### 9. PCB Design & Embedded Systems
**Depth: LIGHT (unless specializing)**

**Why:** Your Teensy integration is solid. Deeper embedded work is a separate specialization.

#### 10. Safety Systems & Standards
**Depth: LEARN ON JOB**

**Why:** ISO 13482 (service robots), functional safety - important but company-specific.

---

## Manipulation: The Other Half of Humanoids

Locomotion is only half the problem. Manipulation (dexterous hands, grasping, tool use) is equally important but uses different techniques:

**Key skills:**
- Grasp planning and contact reasoning
- Imitation learning from human demonstrations
- Tactile sensing and force control
- Bimanual coordination

**Recommended approach:** Focus on locomotion first (Tier 1), then pivot to manipulation. The RL skills transfer directly.

---

## Suggested Timeline (18-24 months)

```
Months 1-4:   State Estimation (EKF/UKF)
              → Add sensor fusion to your AV

Months 4-8:   Trajectory Optimization (MPC)
              → Replace Pure Pursuit with MPC

Months 8-14:  Reinforcement Learning & Sim2Real
              → Train quadruped in Isaac Gym
              → This is the main event

Months 14-18: Dynamics + Real-time
              → Pinocchio, PREEMPT_RT

Months 18-24: First humanoid project
              → Unitree H1 simulation or build custom
```

---

## Recommended Hardware Path

1. **Keep your AV platform** - It's a great testbed for state estimation and MPC
2. **Get a Unitree Go1/Go2** (~$1,600-2,700) - Industry standard for learning legged locomotion
3. **Unitree H1** (when ready) - Most accessible humanoid platform
4. **Or build custom** - Many labs build small bipeds for research

---

## How These Skills Connect

```
State Estimation ←──→ Trajectory Optimization
       ↓                        ↓
       └────→ Reinforcement Learning ←────┘
                      ↓
            Sim2Real Transfer
                      ↓
             Legged Locomotion
                      ↓
             Humanoid Control
                      ↓
               Manipulation
```

Every Tier 1 skill feeds into the others. State estimation gives you belief state for MPC. MPC teaches you constraint handling for reward design. RL policies need dynamics models for sim2real.

---

## Final Thoughts

### What makes you competitive:
1. You've shipped real hardware (rare among students)
2. Your software architecture is production-grade
3. You understand the full sense-plan-act pipeline

### What you need:
1. Mathematical depth in optimization and estimation
2. RL/sim2real for modern locomotion
3. Experience with legged platforms

### Career positioning:
Companies like Figure AI, Tesla Optimus, Agility, and Apptronik hire from:
- Boston Dynamics alumni
- ETH Zurich / MIT / CMU robotics labs
- Self-taught engineers with Isaac Gym experience and real robot demos

Your path: Build a portfolio of simulated + real legged robot demos. A YouTube video of a quadruped you trained from scratch in Isaac Gym is worth more than most resumes.

---

## Sources

- [10 Must-Have Skills for Robotics Engineers 2026](https://www.edstellar.com/blog/robotics-engineers-skills)
- [Learning-based Legged Locomotion - IJRR](https://journals.sagepub.com/doi/10.1177/02783649241312698)
- [Humanoid-Gym: Zero-Shot Sim2Real](https://github.com/roboterax/humanoid-gym)
- [State Estimation for Robotics](https://www.numberanalytics.com/blog/state-estimation-in-robotics-ultimate-guide)
- [MPC vs PID for High-Speed Robots](https://eureka.patsnap.com/article/pid-vs-model-predictive-control-mpc-which-is-better-for-high-speed-robots)
- [LLMs in Robotics - Georgia Tech](https://github.com/GT-RIPL/Awesome-LLM-Robotics)
- [Figure vs Tesla Humanoid Analysis](https://www.diamandis.com/blog/abundance-43-figure-vs-tesla)
- [Dexterous Manipulation Survey](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1682437/full)
