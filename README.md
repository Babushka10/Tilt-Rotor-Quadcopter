Project Title: Simulation of Convertible Octacopter & 8-DOF Tilt-Rotor Quadcopter
Overview: This project explores the design, mathematical modeling, and control of advanced multi-rotor aerial vehicles. Initially conceptualized as a "Convertible Octacopter" for mid-air transformation, the project evolved into a stable 8-DOF Tilt-Rotor Quadcopter. By tilting rotors laterally, this design decouples orientation from movement, allowing the drone to move sideways without rolling.


Key Technical Achievements:

Physics Engine: Developed a custom Newton-Euler dynamics model with Quaternion kinematics to prevent Gimbal Lock.

Control System. The controller is implemented as a cascaded architecture with strict time-scale separation between attitude and position control. Gains for both loops are derived analytically via pole placement, mapping each loop's error dynamics onto a standard second-order characteristic equation (s² + 2ζωₙs + ωₙ² = 0) to set a target damping ratio and natural frequency directly, rather than through iterative tuning. The inner attitude loop is placed at critical damping (ζ = 1.0) for fast, overshoot-free convergence, while the outer position loop is placed at a slightly underdamped ratio (ζ = 0.8) to prioritize tracking responsiveness. A decoupled allocation scheme then splits the commanded lateral force between two actuation paths using a scalar blend parameter β ∈ [0, 1]: at β = 0 all lateral translation is produced by tilting the vehicle body (pitch/roll), at β = 1 it is produced entirely by tilting the rotors while the body stays level, and intermediate values blend the two continuously.


Project Team:

Students:
Parthiv P (Roll No: 132301026)
Abhijith Sureshbabu (Roll No: 102301001) 

Mentors:
Dr. Santhakumar Mohan
Dr. Vijay Muralidharan
