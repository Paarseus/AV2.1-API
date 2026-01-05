# DRIVE Paper - Introduction Draft

## I. INTRODUCTION

The autonomous vehicle industry is experiencing unprecedented growth, creating urgent demand for engineers capable of integrating sensing, computation, and actuation into safety-critical cyber-physical systems. According to the World Economic Forum's 2025 Future of Jobs Report, 63% of employers identify skill gaps as a major barrier to business transformation, with autonomous and electric vehicle specialists ranked among the fastest-growing professions globally [1]. Yet industry leaders consistently report that engineering graduates, while technically competent in isolated domains, lack the systems-level integration experience essential for autonomous vehicle development. A landmark National Academies study on cyber-physical systems education found that Ford Motor Company needs more CPS engineers "than we can get," while NASA's Jet Propulsion Laboratory reported that approximately 80% of new engineering hires require substantial internal development due to incomplete competencies in integrated systems [2]. This gap between academic preparation and industry needs represents a fundamental challenge for engineering education.

### A. The Simulation Paradox

Simulation-based instruction has emerged as the dominant approach to autonomous vehicle education, with platforms like CARLA, LGSVL, and Gazebo enabling students to develop and test algorithms without physical hardware. While these tools democratize access to AV concepts, they fundamentally abstract away the challenges that define real-world autonomous systems development. A comprehensive 2024 meta-analysis of virtual laboratories in engineering education concluded that simulation-based approaches "currently lack the ability to completely replace hands-on labs," recommending hybrid pedagogical strategies that combine virtual and physical experiences [3]. The phenomenon known as the "reality gap" documents systematic differences between simulated and physical systems: real LiDAR sensors suffer from reflections, occlusions, and multipath effects absent in simulation; cameras exhibit lens distortion, motion blur, and exposure variations that simulated sensors ignore; and actuators display deadbands, backlash, and nonlinear dynamics that physics engines simplify or omit [4]. Students trained exclusively in simulation often struggle when confronting physical systems, lacking intuition for sensor noise characteristics, calibration procedures, and the debugging strategies essential for real-world deployment.

### B. The Competition Conundrum

University competitions such as the SAE AutoDrive Challenge and Indy Autonomous Challenge provide authentic autonomous vehicle experiences that simulation cannot replicate. The AutoDrive Challenge, for instance, tasks eight university teams with developing Level 4 autonomous capability on production vehicles, creating close ties between academic programs and industry sponsors [5]. However, these programs require institutional investments exceeding $200,000, dedicate platforms exclusively to competition preparation rather than broad curriculum access, and often rely on proprietary perception or planning stacks that obscure learning opportunities behind black-box interfaces. The competitive pressure inherent in these programs prioritizes performance optimization over pedagogical value, potentially limiting the breadth of student exposure to favor depth in competition-critical areas.

### C. The Scale Tradeoff

Small-scale autonomous platforms have proliferated as cost-effective alternatives to full-scale vehicles. The F1TENTH platform, adopted by over 80 universities worldwide, enables autonomous racing education at 1/10th scale, with studies showing that approximately 80% of students report the hardware platform increases their motivation to learn [6]. Similarly, the Duckietown project, operating at approximately 1/16th scale with ultra-low-cost hardware, has been adopted by over 250 universities since its introduction at MIT in 2016 [7]. The MuSHR platform from the University of Washington offers a complete autonomous racing car for under $1,000, democratizing robotics education further still [8].

Yet these platforms, while valuable for teaching algorithmic concepts, sacrifice critical aspects of full-scale systems engineering. At reduced scale, tire dynamics, actuator response characteristics, and sensor range requirements differ fundamentally from full-scale vehicles. The psychological weight of operating a vehicle capable of causing physical harm—a defining characteristic of safety-critical systems development—is diminished when the platform weighs only a few pounds. Students may develop confidence in approaches that would fail on full-scale systems, where mechanical inertia, braking distances, and failure consequences are orders of magnitude greater.

### D. The Accessibility Barrier

Commercial autonomous vehicle research platforms from companies like AutonomouStuff, or modified production vehicles, provide the authenticity that small-scale platforms lack. However, with costs exceeding $150,000 for a minimally equipped platform, these solutions remain accessible only to well-funded research groups at major institutions, not undergraduate engineering programs seeking to provide broad student access to hands-on autonomous systems experience.

### E. Our Contribution

This paper presents DRIVE (Development Research Infrastructure for Vehicle Education), an autonomous vehicle research and education platform developed at California State Polytechnic University, Pomona. Rather than purchasing a commercial solution or limiting instruction to simulation, we converted a 400-pound youth utility terrain vehicle (UTV) into a complete drive-by-wire research platform. The conversion process itself—designing custom mechanical mounts, developing a distributed CAN bus control architecture, implementing closed-loop control on all actuators, and integrating perception sensors—became a multidisciplinary educational experience spanning mechanical, electrical, and software engineering.

Central to DRIVE is a custom-designed distributed control network built on Controller Area Network (CAN) architecture. Four Teensy 4.1 microcontrollers communicate at 250 kbps to coordinate steering, throttle, and brake actuation, each with encoder feedback enabling true closed-loop control. Unlike approaches that interface with existing vehicle control systems, students designed the entire communication protocol, implemented real-time embedded firmware, and developed multi-layer safety systems including hardware watchdogs, emergency stops, and fail-safe behaviors.

The platform integrates RTK-GPS with centimeter-level positioning, 3D LiDAR, and depth cameras feeding a modular software architecture implementing probabilistic occupancy mapping, costmap generation, and multiple motion planning strategies. A network router provides Ethernet connectivity, allowing any laptop to connect for development while a Jetson Orin AGX provides GPU-accelerated perception when needed. Modular fuseblock-based electronics enable safe isolation and activation of subsystems, designed explicitly for expandability as research needs evolve.

DRIVE occupies a unique position in the landscape of autonomous vehicle education platforms: full-scale authenticity at a fraction of commercial costs, complete accessibility from low-level actuator firmware to high-level planning algorithms, and explicit design for curriculum integration across multiple engineering disciplines. We offer DRIVE as a replicable model for institutions seeking to establish meaningful autonomous vehicle education without the barriers of commercial platform costs or competition program constraints.

The remainder of this paper is organized as follows: Section II describes the mechanical design and drive-by-wire conversion. Section III details the electrical and network architecture. Section IV presents the sensor suite and software architecture. Section V discusses curriculum integration strategies. Section VI presents educational outcomes and lessons learned. Section VII concludes with future directions.

---

## REFERENCES (for Introduction)

[1] World Economic Forum, "The Future of Jobs Report 2025," Geneva, Switzerland, 2025.

[2] National Academies of Sciences, Engineering, and Medicine, "A 21st Century Cyber-Physical Systems Education," Washington, DC: The National Academies Press, 2016.

[3] [Authors], "Effectiveness of Virtual Laboratory in Engineering Education: A Meta-Analysis," PLOS ONE, 2024.

[4] [Authors], "The Reality Gap in Robotics: Challenges, Solutions, and Best Practices," arXiv, 2025.

[5] J. M. Bastiaan, D. L. Peters, J. R. Pimentel, and M. Zadeh, "The AutoDrive Challenge: Autonomous Vehicles Education and Training Issues," in Proc. ASEE Annual Conf. Expo., Tampa, FL, 2019.

[6] J. Betz et al., "F1TENTH: Enhancing Autonomous Systems Education Through Hands-On Learning and Competition," IEEE Trans. Educ., 2024.

[7] L. Paull et al., "Duckietown: An Open, Inexpensive and Flexible Platform for Autonomy Education and Research," in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), 2017.

[8] S. S. Srinivasa et al., "MuSHR: A Low-Cost, Open-Source Robotic Racecar for Education and Research," arXiv:1908.08031, 2019.

---

## Word Count: ~950 words

This introduction is designed to be approximately 1.5-2 pages in IEEE double-column format, which is appropriate for a conference paper introduction.
