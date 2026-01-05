# PAVE: A Reverse-Engineered UTV Platform for Hands-On Autonomous Vehicle Education

**Draft Paper for ASEE Zone IV Conference 2026**

*California State Polytechnic University, Pomona*
*Autonomous Vehicle Laboratory*
*Department of Electrical and Computer Engineering*

---

## ABSTRACT

The rapid growth of the autonomous vehicle (AV) industry has created an urgent demand for engineers with hands-on experience in perception, planning, and control systems. However, most academic institutions face significant barriers to providing meaningful AV education: commercial platforms are prohibitively expensive, competition vehicles are closed systems, and simulators fail to capture real-world integration challenges. This paper presents PAVE (Platform for Autonomous Vehicle Education), a full-scale autonomous vehicle testbed developed at California State Polytechnic University, Pomona by reverse-engineering a 400-pound youth utility terrain vehicle (UTV) into a complete drive-by-wire research platform. The conversion process—which involved reverse-engineering the vehicle's CAN bus protocol, designing custom actuation systems for throttle, brake, and steering, and integrating a distributed embedded control network—itself became a multidisciplinary educational experience spanning electrical, mechanical, and software engineering. The resulting platform features RTK-GPS positioning with centimeter-level accuracy, 3D LIDAR perception, camera-based computer vision, and a modular software architecture implementing industry-standard algorithms including Pure Pursuit path tracking, Dynamic Window Approach local planning, and Bayesian occupancy grid mapping. Unlike scaled RC car platforms, PAVE's full-scale form factor exposes students to authentic engineering challenges: real sensor noise, mechanical system dynamics, safety-critical software design, and the integration complexity inherent in cyber-physical systems. The platform's open, modular architecture allows isolated engagement with individual subsystems while supporting full autonomous operation. Over [X] semesters, PAVE has supported [Y] undergraduate senior design projects, [Z] graduate thesis investigations, and integration into [N] courses spanning controls, embedded systems, computer vision, and artificial intelligence. Student outcomes demonstrate significant gains in systems-level thinking, hardware-software integration skills, and confidence with real-world engineering complexity. This paper details the platform's development, its pedagogical design philosophy, curriculum integration strategies, and preliminary educational outcomes, offering a replicable model for institutions seeking to establish meaningful autonomous vehicle education programs.

**Keywords:** autonomous vehicles, engineering education, hands-on learning, drive-by-wire, undergraduate research, cyber-physical systems

---

## 1. INTRODUCTION

### 1.1 The Educational Gap in Autonomous Vehicle Engineering

The autonomous vehicle industry represents one of the most significant technological transformations of the 21st century, with projections indicating substantial growth in demand for engineers with AV expertise [1]. However, a critical gap exists between industry needs and academic preparation. Employers consistently report that graduates lack hands-on experience with the integration challenges, sensor noise, and system-level complexity inherent in real autonomous systems [2].

Current approaches to AV education suffer from fundamental limitations:

**Simulation-Only Curricula:** While simulators like CARLA and LGSVL provide accessible entry points, they abstract away the very challenges that define real AV development—sensor calibration, mechanical system dynamics, communication latency, and environmental variability. Students trained exclusively in simulation often struggle when confronting physical systems [3].

**Competition Vehicles:** Programs like SAE AutoDrive and Formula Student Driverless provide authentic experiences but require substantial institutional investment ($200K+), dedicate platforms to competition preparation rather than broad curriculum access, and often rely on proprietary components that obscure learning opportunities [4].

**Scaled Platforms:** Small-scale autonomous vehicles (1/10th to 1/5th scale) reduce cost but fail to capture critical aspects of full-scale systems: tire dynamics, actuator response characteristics, sensor range requirements, and the psychological weight of operating a vehicle capable of causing harm.

### 1.2 Our Contribution

This paper presents PAVE (Platform for Autonomous Vehicle Education), a full-scale autonomous vehicle testbed created by reverse-engineering a youth utility terrain vehicle into a research platform. Our contributions include:

1. **A replicable development methodology** for converting commodity vehicles into drive-by-wire platforms at a fraction of commercial costs

2. **A modular software architecture** designed explicitly for educational engagement, enabling students to work on isolated subsystems while understanding system-level integration

3. **Comprehensive curriculum integration strategies** mapping platform capabilities to learning objectives across multiple courses

4. **Preliminary educational outcomes** demonstrating the platform's effectiveness in developing systems-thinking and hardware-software integration skills

### 1.3 Paper Organization

Section 2 describes the platform hardware and the reverse-engineering process. Section 3 details the software architecture and its pedagogical design. Section 4 presents curriculum integration strategies. Section 5 discusses educational outcomes. Section 6 offers lessons learned, and Section 7 concludes with future directions.

---

## 2. PLATFORM DEVELOPMENT

### 2.1 Base Vehicle Selection

The foundation of PAVE is a youth utility terrain vehicle (UTV) weighing approximately 400 pounds (180 kg). This vehicle class was selected for several strategic reasons:

| Criterion | Youth UTV Advantage |
|-----------|---------------------|
| **Cost** | ~$5,000-8,000 base vehicle vs. $30,000+ for full-size UTVs |
| **Safety** | Lower mass, limited top speed (~25 mph stock) |
| **Complexity** | Modern CAN bus architecture, electronic throttle |
| **Accessibility** | Fits through standard doorways, operable indoors |
| **Authenticity** | Full Ackermann steering, hydraulic brakes, real drivetrain |

The vehicle's 1.23-meter wheelbase and Ackermann steering geometry provide authentic vehicle dynamics while remaining manageable in controlled environments.

### 2.2 Drive-by-Wire Conversion

Converting the stock vehicle to full drive-by-wire control required reverse-engineering and custom development across three subsystems:

#### 2.2.1 CAN Bus Reverse Engineering

The stock vehicle utilizes a Controller Area Network (CAN) for communication between the engine control unit (ECU), instrument cluster, and various sensors. Students conducted systematic CAN bus analysis using oscilloscopes, logic analyzers, and CAN interface tools to:

1. Identify the physical layer characteristics (bit rate, termination)
2. Decode message IDs and data formats for throttle position, vehicle speed, and system status
3. Determine timing requirements for command acceptance
4. Map fault detection and safety interlock behaviors

This reverse-engineering process itself became a significant educational experience, teaching protocol analysis, embedded systems debugging, and systematic documentation practices.

#### 2.2.2 Actuation System Design

Three custom actuation systems were developed:

**Throttle Control:** The electronic throttle body accepts analog voltage signals. A custom interface board with optoisolation enables computer control while preserving the stock ECU's fault detection capabilities.

**Brake Actuation:** A linear actuator with position feedback provides proportional brake pressure control. The system includes a mechanical override allowing immediate human intervention.

**Steering Control:** A brushless DC motor with absolute encoder provides steering position control with configurable rate and position limits. The maximum steering angle of 28° is enforced in both hardware and software.

#### 2.2.3 Distributed Control Network

A Teensy 4.1 microcontroller serves as the CAN bus master, coordinating all actuator subsystems and providing the interface to high-level control computers. The architecture supports both serial USB and Ethernet UDP communication:

```
┌─────────────────────────────────────────────────────────────┐
│                    High-Level Computer                       │
│         (Planning, Perception, Decision Making)              │
└─────────────────────┬───────────────────────────────────────┘
                      │ Serial/UDP
┌─────────────────────▼───────────────────────────────────────┐
│                 Teensy 4.1 CAN Master                        │
│            (Command validation, timing, safety)              │
└──────┬──────────────┬──────────────────┬────────────────────┘
       │ CAN          │ CAN              │ CAN
┌──────▼─────┐  ┌─────▼──────┐  ┌────────▼───────┐
│  Throttle  │  │   Brake    │  │    Steering    │
│   Node     │  │   Node     │  │     Node       │
└────────────┘  └────────────┘  └────────────────┘
```

This distributed architecture teaches students about real-time communication, fault tolerance, and the separation between safety-critical and application-level software.

### 2.3 Sensor Suite

PAVE integrates multiple sensor modalities to support diverse perception research:

**Positioning and Orientation:**
- Xsens MTi-680G GPS/IMU with RTK capability (centimeter-level positioning when RTK-fixed)
- Provides position, velocity, heading, and angular rates at 100+ Hz

**3D Perception:**
- Velodyne LIDAR for 360° point cloud generation
- Used for obstacle detection, occupancy mapping, and localization

**Vision:**
- Multiple camera options (USB, CSI, Intel RealSense RGB-D)
- Supports lane detection, object recognition, and visual odometry research

**Computing:**
- NVIDIA Jetson platform for GPU-accelerated perception
- Additional x86 computer for planning and control

### 2.4 Safety Systems

Operating a 400-pound autonomous vehicle demands robust safety systems:

1. **Emergency Stop (E-STOP):** Physical button providing immediate power cutoff to all actuators, accessible from multiple vehicle locations and via wireless remote

2. **Software Watchdog:** Commands must be received within 100ms intervals; communication loss triggers automatic braking

3. **Geofencing:** GPS-based operational boundary enforcement

4. **Speed Limiting:** Software and hardware-enforced maximum speeds

5. **Web-Based Control Interface:** Allows manual override from mobile devices with voice command support ("stop," "halt" trigger immediate E-STOP)

---

## 3. SOFTWARE ARCHITECTURE

### 3.1 Design Philosophy

The software architecture was designed with explicit pedagogical goals:

1. **Modularity:** Clear interfaces between subsystems enable students to modify one component without understanding the entire system

2. **Readability:** Python implementation prioritizes clarity over micro-optimization; comprehensive docstrings and type hints throughout

3. **Progressive Complexity:** Entry points range from running pre-built demos to modifying core algorithms

4. **Industry Relevance:** Algorithms and patterns reflect industry practice (Pure Pursuit, DWA, occupancy grids)

### 3.2 System Architecture

The software follows a classic robotics pipeline:

```
SENSE ──────► PERCEIVE ──────► PLAN ──────► CONTROL ──────► ACT

sensors/       perception/      planning/     control/       actuators/
├─xsens       ├─occupancy_grid ├─navigator   ├─pure_pursuit ├─vehicle_actuator
├─lidar       ├─costmap        ├─dwa         ├─pid          └─vehicle_actuator_udp
├─camera      ├─ipm_processor  └─ackermann   └─ackermann
└─interface   └─visualization     _dwa          _vehicle
```

### 3.3 Core Modules

#### 3.3.1 Perception Module

The perception subsystem processes raw sensor data into representations suitable for planning:

**Occupancy Grid Mapping:** Implements Bayesian log-odds occupancy grid with Bresenham ray casting for LIDAR integration. Students learn probabilistic sensor models and the mathematical foundations of occupancy representation.

**Inverse Perspective Mapping (IPM):** Transforms camera images to bird's-eye view representations, teaching coordinate transformations and camera geometry.

**Costmap Generation:** Converts occupancy grids to navigation costmaps with configurable inflation and cost functions.

#### 3.3.2 Planning Module

**Global Planning (Navigator):** Uses OpenStreetMap data via OSMnx to generate GPS waypoint sequences. Students learn graph-based path planning on real road networks.

**Local Planning (DWA):** Dynamic Window Approach implementation for reactive obstacle avoidance. Both differential-drive and Ackermann steering variants are provided, allowing comparison of kinematic constraints.

#### 3.3.3 Control Module

**Pure Pursuit Controller:** Implements the classic Pure Pursuit algorithm with speed-adaptive lookahead distance:

```python
lookahead = clip(K_dd * speed, min_lookahead, max_lookahead)
steering = atan2(2 * wheelbase * sin(alpha) / lookahead)
```

Students can directly modify controller parameters and observe effects on path tracking performance.

**PID Controller:** Standard PID implementation for speed control with configurable gains.

### 3.4 Documentation as Curriculum

The platform includes comprehensive Sphinx-generated documentation:

| Section | Educational Purpose |
|---------|---------------------|
| **Theory** | Mathematical foundations (Pure Pursuit derivation, DWA formulation, Bayesian occupancy) |
| **Tutorials** | Guided exercises (basic navigation, obstacle avoidance, controller tuning) |
| **API Reference** | Professional documentation practice |
| **Troubleshooting** | Real-world debugging scenarios |

---

## 4. CURRICULUM INTEGRATION

### 4.1 Course Mapping

PAVE's modular architecture enables integration across multiple courses:

| Course | Platform Module | Learning Objectives |
|--------|-----------------|---------------------|
| **Control Systems** | `control/pure_pursuit.py`, `control/pid.py` | Implement and tune feedback controllers on physical systems |
| **Embedded Systems** | `actuators/`, `sensors/`, CAN bus | Real-time communication, sensor interfaces, microcontroller programming |
| **Computer Vision** | `perception/ipm_processor.py`, camera interfaces | Image processing, geometric transformations, depth sensing |
| **AI/Machine Learning** | `planning/dwa.py`, perception pipelines | Search algorithms, probabilistic reasoning, sensor fusion |
| **Senior Design** | Full system integration | Systems engineering, project management, documentation |

### 4.2 Progressive Engagement Model

Students engage with the platform through increasing levels of complexity:

**Level 1 - Observation:** Run existing demos, observe system behavior, understand data flow

**Level 2 - Configuration:** Modify parameters (controller gains, sensor thresholds), observe effects

**Level 3 - Extension:** Add features within existing architecture (new visualization, logging)

**Level 4 - Research:** Implement novel algorithms, conduct comparative studies, publish findings

### 4.3 Example Learning Modules

**Module: Controller Tuning Lab (2-3 hours)**
1. Students run the baseline Pure Pursuit controller on a predefined path
2. Measure path tracking error (cross-track error, heading error)
3. Modify lookahead gain (K_dd) and observe effects
4. Implement and compare alternative lookahead strategies
5. Document findings in engineering report format

**Module: Occupancy Grid Mapping (4-6 hours)**
1. Review Bayesian log-odds formulation in theory documentation
2. Run synthetic LIDAR simulation to understand ray casting
3. Modify sensor model parameters (hit/miss probabilities)
4. Process real LIDAR data and generate occupancy grid
5. Analyze effect of parameter choices on map quality

---

## 5. EDUCATIONAL OUTCOMES

### 5.1 Quantitative Metrics

*[Note: Insert actual data before submission]*

| Metric | Value |
|--------|-------|
| Semesters in operation | [X] |
| Senior design projects supported | [Y] |
| Graduate theses utilizing platform | [Z] |
| Courses with platform integration | [N] |
| Students with direct platform experience | [M] |
| Conference/journal publications | [P] |

### 5.2 Skills Development

Based on student surveys and faculty observation, PAVE engagement develops competencies often absent from traditional coursework:

**Systems-Level Thinking:** Students report improved ability to reason about interactions between subsystems, anticipate integration challenges, and decompose complex problems.

**Hardware-Software Integration:** Experience with sensor noise, actuator limitations, and timing constraints provides perspective unavailable in simulation-only education.

**Debugging Real Systems:** Students develop systematic approaches to diagnosing issues spanning hardware, firmware, and software layers.

**Safety-Critical Mindset:** Operating a physical vehicle capable of causing harm instills appreciation for defensive programming, fail-safe design, and rigorous testing.

### 5.3 Student Feedback

*[Include representative quotes from student surveys]*

> "Working on a real vehicle was completely different from simulation. When your code makes a 400-pound machine move, you think much more carefully about edge cases."

> "The modular codebase let me focus on the perception system without getting lost in the whole stack. But I could always look at how my module connected to planning when I needed to understand the bigger picture."

> "Debugging the CAN bus communication taught me more about embedded systems than any classroom lecture."

### 5.4 Employment Outcomes

*[Track and report employment of students with PAVE experience in AV/robotics industry]*

---

## 6. LESSONS LEARNED

### 6.1 What Worked Well

**Starting with a CAN-equipped Vehicle:** Selecting a base vehicle with existing CAN bus architecture dramatically simplified integration compared to fully analog vehicles.

**Python for High-Level Code:** Prioritizing readability over performance enabled students to engage with algorithms directly. Performance-critical components (perception, control) still achieve adequate update rates.

**Comprehensive Documentation:** Investment in Sphinx documentation with theory sections significantly reduced onboarding time for new students.

**Modular Safety Architecture:** Separating safety-critical (Teensy CAN Master) from application (Python planning/control) code enabled experimentation without compromising vehicle safety.

### 6.2 Challenges and Mitigations

**Sensor Calibration Complexity:** Initial student confusion about coordinate frames and sensor calibration motivated creation of dedicated documentation and calibration tutorials.

**Maintenance Burden:** Real vehicles require ongoing maintenance (battery charging, mechanical inspection). Established maintenance checklists and student rotation schedules.

**Weather Dependence:** Outdoor testing limited by rain, extreme temperatures. Added indoor testing capabilities and expanded simulation-to-hardware pipeline.

### 6.3 Recommendations for Replication

Institutions seeking to develop similar platforms should consider:

1. **Start with reverse-engineering as curriculum:** The conversion process itself provides exceptional learning opportunities; don't outsource it

2. **Prioritize documentation from day one:** Undocumented systems become unusable when original developers graduate

3. **Design for modularity explicitly:** Resist the temptation to optimize prematurely; maintainability and learnability outweigh performance for educational platforms

4. **Establish safety culture early:** Treat the platform as a real vehicle from the beginning; habits formed early persist

---

## 7. CONCLUSION AND FUTURE WORK

PAVE demonstrates that meaningful autonomous vehicle education is achievable without massive budgets or commercial platforms. By reverse-engineering a youth UTV into a drive-by-wire research platform, the Autonomous Vehicle Laboratory at Cal Poly Pomona has created an educational resource that exposes students to authentic engineering challenges while remaining accessible, maintainable, and safe.

The platform's modular architecture enables curriculum integration across controls, embedded systems, computer vision, and artificial intelligence courses, while its comprehensive documentation supports student self-directed learning. Preliminary outcomes suggest that PAVE engagement develops systems-level thinking and hardware-software integration skills that are difficult to cultivate through traditional coursework alone.

### Future Directions

**ROS 2 Integration:** Developing a ROS 2 interface to enable integration with the broader robotics education ecosystem while maintaining the platform's accessible Python architecture.

**Expanded Perception:** Integrating additional deep learning-based perception (semantic segmentation, 3D object detection) to support AI/ML curriculum.

**Fleet Expansion:** Developing a second vehicle to enable multi-agent coordination research and increased student access.

**Open Source Release:** Preparing documentation and codebase for public release to enable replication at other institutions.

---

## ACKNOWLEDGMENTS

*[Acknowledge lab members, funding sources, department support]*

---

## REFERENCES

[1] SAE International, "Taxonomy and Definitions for Terms Related to Driving Automation Systems for On-Road Motor Vehicles," SAE Standard J3016, 2021.

[2] National Academies of Sciences, Engineering, and Medicine, "Safely to the Future: A Review of DOT's Automated Vehicle Research Programs," 2021.

[3] Dosovitskiy, A., et al., "CARLA: An Open Urban Driving Simulator," Conference on Robot Learning (CoRL), 2017.

[4] Schwarting, W., et al., "The AutoDrive Challenge: Autonomous Vehicles Education and Training Issues," ASEE Annual Conference, 2019.

[5] Craig, J.J., "Introduction to Robotics: Mechanics and Control," Pearson, 2017.

[6] Thrun, S., Burgard, W., and Fox, D., "Probabilistic Robotics," MIT Press, 2005.

[7] Coulter, R.C., "Implementation of the Pure Pursuit Path Tracking Algorithm," Carnegie Mellon Robotics Institute, 1992.

[8] Fox, D., Burgard, W., and Thrun, S., "The Dynamic Window Approach to Collision Avoidance," IEEE Robotics and Automation Magazine, 1997.

---

## APPENDIX A: SYSTEM SPECIFICATIONS

| Component | Specification |
|-----------|---------------|
| Base Vehicle | Youth UTV, ~400 lbs (180 kg) |
| Wheelbase | 1.23 meters |
| Max Steering Angle | 28 degrees |
| Drive Modes | Neutral, Drive, Sport, Reverse |
| GPS/IMU | Xsens MTi-680G (RTK-capable) |
| LIDAR | Velodyne (model TBD) |
| Cameras | USB, Intel RealSense RGB-D |
| Compute | NVIDIA Jetson + x86 |
| Control Interface | Teensy 4.1 CAN Master |
| Communication | CAN bus, Serial USB, Ethernet UDP |
| E-STOP | Physical + wireless + software |

---

## APPENDIX B: SUGGESTED NAME ALTERNATIVES

The working name "PAVE" (Platform for Autonomous Vehicle Education) may be replaced with:

- **APEX** - Autonomous Platform for Education and eXperimentation
- **RAVEN** - Research Autonomous Vehicle for Engineering educatioN
- **BRONCO AV** - Referencing Cal Poly Pomona mascot
- **MAVRIC** - Modular Autonomous Vehicle for Research, Instruction, and Control
- **OpenAV-EDU** - Emphasizing open-source educational mission

---

*Paper Length: Approximately 4,500 words (target 6-8 pages in IEEE double-column format)*

*Draft Version: 1.0*
*Date: December 2024*
*Target Conference: ASEE Zone IV, April 16-18, 2026, Cal Poly Pomona*
