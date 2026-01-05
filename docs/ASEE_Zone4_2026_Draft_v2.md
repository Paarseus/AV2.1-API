# DRIVE: Bridging Theory and Practice in Autonomous Vehicle Education

**Draft Paper for ASEE Zone IV Conference 2026**

*California State Polytechnic University, Pomona*
*Autonomous Vehicle Laboratory*
*College of Engineering*

---

## ABSTRACT

The autonomous vehicle industry's rapid expansion has created unprecedented demand for engineers who can integrate sensing, computation, and actuation into safety-critical cyber-physical systems. Yet most academic programs struggle to provide authentic hands-on experience: simulators abstract away real-world complexity, competition platforms are prohibitively expensive and closed, and small-scale vehicles fail to capture the engineering challenges of full-scale systems. This paper presents DRIVE (Development Research Infrastructure for Vehicle Education), an open-architecture autonomous vehicle research platform developed by converting a 400-pound utility terrain vehicle into a fully instrumented drive-by-wire testbed. Central to the platform is a custom-designed distributed control network built on CAN bus architecture, where students designed the communication protocol, implemented real-time embedded firmware on Teensy microcontrollers, and developed multi-layer safety systems including hardware watchdogs, emergency stops, and fail-safe behaviors. The sensor suite—comprising RTK-GPS with centimeter-level positioning, 3D LIDAR, and vision systems—feeds a modular perception pipeline implementing probabilistic occupancy mapping, costmap generation, and camera-based scene understanding. The platform has served as a testbed for multiple motion planning and control strategies, from graph-based global routing to reactive local planners to geometric path followers, enabling comparative algorithm studies impossible on single-purpose platforms. Unlike commercial research vehicles, every layer of DRIVE—from low-level actuator firmware to high-level planning algorithms—is accessible, documented, and modifiable, transforming the vehicle itself into curriculum. Students engage across the full autonomy stack: electrical engineers design sensor interfaces and power systems; computer engineers develop embedded control nodes; software engineers architect perception pipelines; and all collaborate on system integration. This paper details the platform's technical architecture, its deliberate design for educational engagement, strategies for curriculum integration across multiple courses, and preliminary outcomes demonstrating development of systems-thinking competencies rarely achieved through traditional instruction. We offer DRIVE as a replicable model for institutions seeking to establish meaningful autonomous vehicle education without the barriers of commercial platform costs or competition program constraints.

---

**Keywords:** autonomous vehicles, engineering education, drive-by-wire, CAN bus, embedded systems, hands-on learning, cyber-physical systems, undergraduate research

---

## 1. INTRODUCTION

### 1.1 The Workforce Challenge

The autonomous vehicle industry represents a convergence of mechanical, electrical, computer, and software engineering disciplines unlike any previous technological domain. Industry surveys consistently identify a skills gap: graduates possess theoretical knowledge in isolated domains but lack experience integrating sensors, actuators, embedded systems, and algorithms into functioning cyber-physical systems [1]. The complexity of autonomous vehicles—where a software bug can cause physical harm—demands engineers who think in systems, understand failure modes across domains, and appreciate the constraints that real hardware imposes on elegant algorithms.

### 1.2 Limitations of Current Approaches

Academic institutions have attempted various approaches to AV education, each with significant limitations:

**Simulation-Only Curricula**: Platforms like CARLA, LGSVL, and MATLAB provide accessible entry points but fundamentally abstract away the challenges that define real AV development. Students never experience sensor noise, calibration drift, actuator latency, or the gap between simulated and real-world dynamics. When these students encounter physical systems, they often struggle with debugging strategies and lack intuition for hardware-software interaction [2].

**Competition Vehicles**: Programs like SAE AutoDrive Challenge and Indy Autonomous Challenge provide authentic experiences but require institutional investments exceeding $200,000, dedicate platforms to competition preparation rather than broad curriculum access, and often rely on proprietary perception or planning stacks that obscure learning opportunities. The competitive pressure also prioritizes performance over pedagogical value [3].

**Small-Scale Platforms**: Vehicles at 1/10th to 1/5th scale (F1TENTH, MIT RACECAR, DonkeyCar) reduce costs but sacrifice critical aspects of full-scale engineering. Tire dynamics, actuator response characteristics, sensor range requirements, and the psychological weight of operating a vehicle capable of causing harm are all diminished. Students may develop false confidence from platforms that forgive errors real vehicles would not [4].

**Commercial Research Platforms**: Vehicles from companies like AutonomouStuff or modified production vehicles provide authenticity but at costs ($150,000+) that limit access to well-funded research groups, not undergraduate education.

### 1.3 Our Approach

This paper presents DRIVE (Development Research Infrastructure for Vehicle Education), an autonomous vehicle research platform developed at California State Polytechnic University, Pomona. Rather than purchasing a commercial solution, we converted a 400-pound youth utility terrain vehicle into a complete drive-by-wire research platform. The conversion process—designing the CAN bus architecture, developing actuator control nodes, implementing safety systems—itself became a multidisciplinary educational experience.

The resulting platform provides:

1. **Full-scale authenticity** at a fraction of commercial costs
2. **Complete accessibility** from low-level firmware to high-level algorithms
3. **Algorithm flexibility** enabling comparative studies across planning and control approaches
4. **Educational design** with modular architecture supporting isolated and integrated learning
5. **Research capability** sufficient for graduate-level investigation and publication

### 1.4 Contributions

This paper makes the following contributions:

- Technical architecture of a custom-designed drive-by-wire control system using commodity microcontrollers and CAN bus
- A modular software architecture explicitly designed for educational engagement
- Curriculum integration strategies mapping platform capabilities to learning objectives
- Preliminary educational outcomes and lessons learned
- A replicable model for institutions seeking affordable AV education platforms

---

## 2. PLATFORM ARCHITECTURE

### 2.1 Base Vehicle

DRIVE is built on a youth utility terrain vehicle (UTV) weighing approximately 400 pounds (180 kg). This vehicle class offers an optimal balance for educational purposes:

| Factor | Advantage |
|--------|-----------|
| **Cost** | $5,000-8,000 base vehicle vs. $30,000+ for full-size UTVs |
| **Scale** | Full Ackermann steering geometry, hydraulic brakes, real drivetrain |
| **Safety** | Lower mass and speed capability than full-size vehicles |
| **Accessibility** | Fits through standard doorways, operable in parking lots |
| **Modern Electronics** | Electronic throttle, suitable for drive-by-wire conversion |

The 1.23-meter wheelbase and 28-degree maximum steering angle provide authentic vehicle dynamics while remaining manageable in controlled environments.

### 2.2 Custom Drive-by-Wire Architecture

The heart of DRIVE is a custom-designed distributed control system built on Controller Area Network (CAN) architecture. Unlike approaches that interface with existing vehicle control systems, we designed the entire control network from first principles.

#### 2.2.1 Distributed Node Architecture

The system comprises four Teensy 4.1 microcontrollers communicating over CAN bus at 250 kbps:

```
┌─────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL INTERFACES                         │
│         Serial USB │ Ethernet UDP │ WebSocket │ Voice           │
└─────────────────────────────┬───────────────────────────────────┘
                              │
┌─────────────────────────────▼───────────────────────────────────┐
│                     MASTER NODE (Teensy 4.1)                     │
│     • Command parsing and validation                             │
│     • Protocol translation                                       │
│     • System state monitoring                                    │
│     • Safety arbitration                                         │
└─────────────────────────────┬───────────────────────────────────┘
                              │ CAN Bus (250 kbps)
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
┌───────▼───────┐     ┌───────▼───────┐     ┌───────▼───────┐
│   THROTTLE    │     │    STEER      │     │    BRAKE      │
│   (0x100)     │     │   (0x200)     │     │   (0x300)     │
├───────────────┤     ├───────────────┤     ├───────────────┤
│ MCP4728 DAC   │     │ Stepper Motor │     │ Linear        │
│ Mode Select   │     │ Abs. Encoder  │     │ Actuator      │
│ 4-channel     │     │ Limit Switch  │     │ Position FB   │
└───────────────┘     └───────────────┘     └───────────────┘
```

Each subsystem node operates autonomously with its own control loop, enabling graceful degradation and simplified debugging.

#### 2.2.2 Custom Protocol Design

Students designed the CAN message protocol, making deliberate choices about message structure, timing, and safety:

**Command Messages (Master → Nodes):**
| CAN ID | Payload | Description |
|--------|---------|-------------|
| 0x100 | E-STOP, Throttle (0-255), Mode (N/D/S/R) | Throttle control |
| 0x200 | Center flag, Steering (-127 to +127) | Steering control |
| 0x300 | E-STOP, Brake (0-255) | Brake control |

**Status Messages (Nodes → Master):**
| CAN ID | Payload | Description |
|--------|---------|-------------|
| 0x101 | DAC value, mode, voltage | Throttle feedback |
| 0x201 | Encoder ADC, limit flags, setpoint | Steering feedback |
| 0x301 | Actuator position, command state | Brake feedback |

The protocol includes the E-STOP flag in every command message, ensuring immediate safety response regardless of which subsystem receives the command first.

#### 2.2.3 Multi-Layer Safety Architecture

Safety-critical system design is a core learning objective. DRIVE implements defense-in-depth:

**Layer 1 - Hardware:**
- Physical E-STOP buttons (vehicle-mounted and wireless remote)
- Mechanical limit switches on steering
- Actuator stroke limits

**Layer 2 - Firmware Watchdogs:**
- 100-200ms command timeout triggers fail-safe behavior
- Brake: automatic full application
- Throttle: return to neutral, idle voltage
- Steering: hold current position

**Layer 3 - Protocol:**
- E-STOP flag in every CAN message
- Status feedback for health monitoring
- No command broadcasting (targeted messages only)

**Layer 4 - Application:**
- Maximum throttle limits (configurable)
- Geofencing via GPS
- Single-controller enforcement (prevents conflicts)

### 2.3 Actuator Subsystems

Each actuator presents distinct engineering challenges:

**Throttle System**: A 4-channel 12-bit DAC (MCP4728) generates analog voltages for the electronic throttle body and transmission mode selection. Students learn I2C communication, analog signal conditioning, and the relationship between DAC resolution and control precision.

**Steering System**: A stepper motor with absolute encoder provides position control. Students implement non-blocking motion control, calibration procedures, and position feedback loops with configurable deadbands.

**Brake System**: A linear actuator with analog position feedback provides proportional braking. Students develop understanding of closed-loop position control, actuator dynamics, and fail-safe design (brakes apply on power loss).

### 2.4 Sensor Integration

DRIVE integrates multiple sensor modalities:

**Positioning and Orientation:**
- Xsens MTi-680G GPS/INS with RTK capability
- Centimeter-level positioning when RTK-fixed
- 100+ Hz position, velocity, and orientation updates
- ENU coordinate frame output

**3D Perception:**
- Velodyne LIDAR for 360° point cloud generation
- Real-time obstacle detection and mapping

**Vision:**
- USB and Intel RealSense RGB-D cameras
- Support for monocular and depth-based perception

**Computing:**
- NVIDIA Jetson for GPU-accelerated perception
- x86 system for planning and control
- Ethernet and USB connectivity throughout

### 2.5 Interface Options

Multiple interfaces enable different use cases:

| Interface | Use Case | Protocol |
|-----------|----------|----------|
| Serial USB | Direct laptop control, debugging | ASCII commands |
| Ethernet UDP | Low-latency autonomous control | ASCII commands |
| WebSocket + HTTPS | Mobile device control, demos | JSON over WebSocket |
| Voice Commands | Accessibility, hands-free operation | Speech → command mapping |

The web interface includes a touch joystick, E-STOP button, and voice recognition (via Whisper), enabling intuitive manual control from any smartphone.

---

## 3. SOFTWARE ARCHITECTURE

### 3.1 Design Principles

The software architecture reflects explicit pedagogical goals:

**Modularity**: Clear interfaces between subsystems enable students to modify one component without understanding the entire codebase. A student working on perception need not understand actuator firmware.

**Readability**: Python implementation prioritizes clarity. Comprehensive type hints and docstrings make the code self-documenting. Performance-critical sections remain readable while achieving adequate real-time performance.

**Progressive Complexity**: Entry points range from running pre-built demonstrations to implementing novel algorithms. Students can engage at their skill level and grow into deeper understanding.

**Industry Relevance**: Algorithms and patterns reflect industry practice, preparing students for professional work.

### 3.2 Modular Pipeline

The software follows a classic robotics architecture:

```
┌─────────┐    ┌───────────┐    ┌──────────┐    ┌─────────┐    ┌──────────┐
│ SENSORS │───▶│ PERCEPTION│───▶│ PLANNING │───▶│ CONTROL │───▶│ ACTUATORS│
└─────────┘    └───────────┘    └──────────┘    └─────────┘    └──────────┘
     │              │                │               │              │
  xsens         occupancy         navigator      pure_pursuit   CAN Master
  lidar         costmap           local          pid            Teensy
  camera        IPM/BEV           planners       kinematics     nodes
```

Each module can be developed, tested, and modified independently.

### 3.3 Perception Capabilities

**Occupancy Grid Mapping**: Bayesian log-odds formulation with Bresenham raycasting for free space. Students learn probabilistic sensor models and the mathematical foundations of spatial reasoning.

**Costmap Generation**: Pre-inflated obstacle representation enabling efficient collision checking. Demonstrates the concept of configuration space.

**Inverse Perspective Mapping**: Camera-to-ground-plane transformation for bird's eye view generation. Teaches coordinate transformations and camera geometry.

**Deep Learning Integration**: Adapter for multi-task perception (detection, lane segmentation, drivable area) demonstrating neural network integration with classical robotics.

### 3.4 Planning Algorithms

The platform has been used to implement and compare multiple planning approaches:

**Global Planning**: Graph-based routing on OpenStreetMap road networks, generating waypoint sequences for long-range navigation.

**Local Planning**: Reactive planners for obstacle avoidance, including variants optimized for different vehicle kinematics (differential drive, Ackermann steering) and different obstacle representations (raw points, inflated costmaps).

**Path Following**: Geometric controllers with adaptive parameters, enabling smooth trajectory tracking at varying speeds.

This diversity enables comparative studies: Which planner performs better in cluttered environments? How do different lookahead strategies affect tracking accuracy? These questions become tractable experiments rather than theoretical exercises.

### 3.5 Control Implementation

**Geometric Path Following**: Implements adaptive lookahead based on vehicle velocity, with configurable gain parameters students can tune experimentally.

**Feedback Control**: Standard PID implementation for velocity regulation, with anti-windup and configurable gains.

**Vehicle Kinematics**: Bicycle model implementation providing trajectory prediction and turning radius calculation.

### 3.6 Visualization and Debugging

Real-time visualization aids development and debugging:

- **3D Visualization**: Multi-layer viewer combining camera projections, LIDAR point clouds, occupancy grids, and planned paths
- **2D Visualization**: Matplotlib-based viewers for grids, costmaps, and GPS trails
- **Route Visualization**: Real-time position tracking on street maps

---

## 4. CURRICULUM INTEGRATION

### 4.1 Multidisciplinary Engagement

DRIVE's architecture enables engagement across engineering disciplines:

| Discipline | Platform Engagement |
|------------|---------------------|
| **Electrical Engineering** | Sensor interfaces, power distribution, signal conditioning, CAN bus physical layer |
| **Computer Engineering** | Embedded firmware, real-time systems, communication protocols, FPGA opportunities |
| **Mechanical Engineering** | Actuator selection, mechanism design, vehicle dynamics, sensor mounting |
| **Computer Science** | Perception algorithms, planning systems, software architecture |
| **Systems Engineering** | Integration, testing, documentation, safety analysis |

### 4.2 Course Mapping

Specific courses can leverage platform subsystems:

| Course | Platform Module | Learning Objectives |
|--------|-----------------|---------------------|
| Control Systems | Path following, PID | Implement and tune controllers on physical systems |
| Embedded Systems | CAN nodes, firmware | Real-time programming, communication protocols |
| Computer Vision | Camera pipeline, IPM | Image processing, geometric transformations |
| AI/Robotics | Planning, perception | Search algorithms, probabilistic reasoning |
| Senior Design | Full integration | Systems engineering, project management |

### 4.3 Progressive Learning Model

Students engage through increasing levels of complexity:

**Level 1 - Observation**: Run existing demonstrations, observe system behavior, trace data flow through logs and visualization.

**Level 2 - Configuration**: Modify parameters (controller gains, sensor thresholds, planner weights), observe and measure effects.

**Level 3 - Extension**: Add features within existing architecture (new visualization, additional logging, parameter optimization).

**Level 4 - Research**: Implement novel algorithms, conduct comparative studies, contribute to publications.

### 4.4 Learning Modules

Example structured activities:

**Module: CAN Bus Communication (4-6 hours)**
1. Review CAN protocol fundamentals
2. Analyze existing message definitions using logic analyzer
3. Implement new status message on a subsystem node
4. Verify message reception and parsing on Master
5. Document protocol extension

**Module: Planner Comparison (6-8 hours)**
1. Run baseline planner in simulation environment
2. Measure performance metrics (path length, smoothness, computation time)
3. Implement alternative planner from literature
4. Compare performance under identical conditions
5. Analyze trade-offs in technical report

---

## 5. OUTCOMES AND DISCUSSION

### 5.1 Research Flexibility

DRIVE has demonstrated value as a research platform:

- Comparative studies across planning algorithms with identical hardware
- Perception algorithm development with real sensor data
- Control strategy evaluation under realistic conditions
- Graduate thesis investigations in navigation and perception

The platform's openness enables research questions that closed systems cannot address.

### 5.2 Educational Observations

Students engaging with DRIVE develop competencies difficult to cultivate through traditional instruction:

**Systems Thinking**: Understanding how subsystem interactions create emergent behaviors—how perception latency affects control stability, how actuator limits constrain planning, how safety requirements shape architecture.

**Debugging Real Systems**: Developing systematic approaches to problems spanning hardware, firmware, and software. Learning to isolate issues across domain boundaries.

**Safety-Critical Mindset**: Appreciating defensive programming, fail-safe design, and rigorous testing when software controls a physical vehicle.

**Hardware-Software Integration**: Experiencing sensor noise, actuator limitations, and timing constraints provides perspective unavailable in simulation.

### 5.3 Student Feedback

*[Representative quotes to be added from student surveys]*

> "Designing the CAN protocol taught me more about embedded systems than any classroom lecture. When your message format has to work reliably or the vehicle doesn't stop, you think very carefully about edge cases."

> "I finally understood why sensor fusion matters when I saw how noisy the raw LIDAR data was. In simulation, everything is perfect. Here, nothing is."

### 5.4 Replication Considerations

Institutions considering similar platforms should note:

**Advantages of Our Approach:**
- Total hardware cost approximately $15,000-20,000 (vehicle, sensors, compute, actuators)
- No proprietary dependencies; all firmware and software developed in-house
- Scalable complexity; can start with manual control and incrementally add autonomy
- Maintenance addressable by students, building additional skills

**Challenges:**
- Initial development requires faculty/student expertise across domains
- Ongoing maintenance (battery, mechanical wear) requires institutional support
- Outdoor testing weather-dependent; indoor capabilities valuable
- Safety protocols and risk management require institutional buy-in

---

## 6. CONCLUSION

DRIVE demonstrates that meaningful autonomous vehicle education is achievable without massive budgets or commercial dependencies. By designing a complete drive-by-wire control system and implementing the full autonomy stack on a converted utility vehicle, the Autonomous Vehicle Laboratory at Cal Poly Pomona has created a platform that serves both education and research.

The platform's key strengths for education are its openness and accessibility. Every layer—from CAN bus firmware to planning algorithms—is visible, modifiable, and documented. Students don't just use an autonomous vehicle; they understand how it works and can change how it works. This depth of engagement develops engineering intuition that commercial platforms, by their nature, cannot provide.

For research, the platform's flexibility enables studies across algorithms and approaches without the constraints of competition rules or proprietary limitations. The ability to implement, test, and compare different strategies on identical hardware creates opportunities for rigorous experimental research.

We offer DRIVE as a model for institutions seeking to establish autonomous vehicle education programs. The technical challenges are real but surmountable. The educational rewards—students who think in systems, debug across domains, and understand the full complexity of cyber-physical systems—are substantial.

### Future Work

- ROS 2 interface development for ecosystem integration
- Expanded deep learning perception capabilities
- Multi-vehicle coordination for fleet research
- Open-source release of firmware and software
- Formal curriculum module development with assessment instruments

---

## REFERENCES

[1] National Academies of Sciences, Engineering, and Medicine, "Safely to the Future: A Review of DOT's Automated Vehicle Research Programs," 2021.

[2] Dosovitskiy, A., et al., "CARLA: An Open Urban Driving Simulator," Conference on Robot Learning, 2017.

[3] Schwarting, W., et al., "The AutoDrive Challenge: Autonomous Vehicles Education and Training Issues," ASEE Annual Conference, 2019.

[4] O'Kelly, M., et al., "F1TENTH: An Open-Source Evaluation Environment for Continuous Control and Reinforcement Learning," NeurIPS Competition Track, 2019.

[5] Thrun, S., Burgard, W., and Fox, D., "Probabilistic Robotics," MIT Press, 2005.

[6] Texas Instruments, "Introduction to the Controller Area Network (CAN)," Application Report SLOA101B, 2016.

---

## APPENDIX A: PLATFORM SPECIFICATIONS

| Component | Specification |
|-----------|---------------|
| Base Vehicle | Youth UTV, ~400 lbs (180 kg) |
| Wheelbase | 1.23 m |
| Max Steering | 28° |
| Drive Modes | Neutral, Drive, Sport, Reverse |
| CAN Bus | 250 kbps, FlexCAN_T4 |
| Microcontrollers | 4× Teensy 4.1 |
| GPS/INS | Xsens MTi-680G (RTK-capable) |
| LIDAR | Velodyne VLP-16 |
| Cameras | USB, Intel RealSense |
| Compute | NVIDIA Jetson + x86 |
| Safety | Physical E-STOP, wireless E-STOP, firmware watchdogs |

---

## APPENDIX B: NAME

**DRIVE** — Development Research Infrastructure for Vehicle Education

Other options considered:
- **APEX** — Autonomous Platform for Education and eXperimentation
- **PAVE** — Platform for Autonomous Vehicle Education
- **FORGE** — Framework for Open Research in Ground-vehicle Engineering

---

*Draft Version: 2.0*
*Date: December 2024*
*Target: ASEE Zone IV Conference, April 16-18, 2026, Cal Poly Pomona*
*Word Count: ~4,200 (target 6-8 pages IEEE format)*
