Project Title: Simulation of Convertible Octacopter & 8-DOF Tilt-Rotor Quadcopter
Overview: This project explores the design, mathematical modeling, and control of advanced multi-rotor aerial vehicles. Initially conceptualized as a "Convertible Octacopter" for mid-air transformation, the project evolved into a stable 8-DOF Tilt-Rotor Quadcopter. By tilting rotors laterally, this design decouples orientation from movement, allowing the drone to move sideways without rolling.


Key Technical Achievements:

Physics Engine: Developed a custom Newton-Euler dynamics model with Quaternion kinematics to prevent Gimbal Lock.

Control System: Implemented a Cascaded Control Architecture with Time-Scale Separation. The controller uses Analytic Pole Placement to mathematically derive gains for guaranteed stability (Critically Damped Attitude Loop, Underdamped Position Loop).

Actuation Logic: Created a unique mixing matrix and geometric projection algorithm to manage the 8 degrees of freedom (4 motors + 4 tilt servos).


Project Team:

Students:
Parthiv P (Roll No: 132301026)
Abhijith Sureshbabu (Roll No: 102301001) 

Mentors:
Dr. Santhakumar Mohan
Dr. Vijay Muralidharan
