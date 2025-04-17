Robotic Welding Arm Simulation

This project simulates a 6-DOF robotic welding manipulator for motorcycle headstock manufacturing using MATLAB-based modeling and control strategies. The manipulator is designed to follow a weld sequence across defined points within a constrained 3D workspace. The project is structured in four phases, aligning with ECE 9053A coursework at Western University.

Project Scope

The goal is to automate welding operations by modeling the robot's forward and inverse kinematics, implementing obstacle-aware path planning, and applying a dynamic control system for smooth trajectory execution.

Application: MIG welding of motorcycle headstock frame

Robot Type: 6-DOF articulated arm with 5 revolute joints

Workspace: 2.75m x 2.25m x 0.1m (X, Y, Z)

Tools: MATLAB, Simulink, Optimization Toolbox

Repository Structure

Problem_Statements/ – Original task descriptions from Step 1 to Step 4

Implementation/ – Phase-wise project submissions and results

MATLAB/ – Code files for all modeling, planning, and control tasks

Cad model/ – STEP or STL files representing the geometry of the welding cell and robot

Features

Kinematic Modeling:

Forward and inverse kinematics implemented in MATLAB

Jacobian calculation for velocity analysis

Path Planning:

Potential Field algorithm for obstacle avoidance

Visualization of weld point transitions

Dynamics & Control:

Euler-Lagrange formulation for dynamic equations

Inverse dynamics control with trajectory simulation via ODE45

Validation:

Code verification through inverse-forward loop

Trajectory validation plots for torque and position

Tools Used

MATLAB R2024a

Symbolic and Optimization Toolbox

Simulink (for validation and simulation)

Author

Sannjay BalajiECE 9053A - Robotics and ControlWestern University, Fall 2024