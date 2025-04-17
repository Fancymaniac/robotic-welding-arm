# 🤖 Robotic Welding Arm Simulation

This repository presents a comprehensive simulation of a **6-DOF robotic manipulator** for motorcycle headstock welding. Developed using MATLAB and Simulink, the robot is designed to follow a weld path within a constrained workspace, navigating through obstacles while ensuring smooth joint trajectories.

> 📚 Course: ECE 9053A – Robotics and Control  
> 🏫 Institution: Western University  
> 🛠️ Tools: MATLAB, Simulink, Optimization Toolbox

---

## 📌 Project Scope

The primary objective is to automate welding tasks by:

- Modeling **forward and inverse kinematics** of a 6-DOF articulated arm
- Implementing **potential field-based path planning**
- Simulating **dynamic control** using the Euler-Lagrange formulation
- Generating optimized joint torque profiles for real-time execution

### ✳️ Application

- **Task**: MIG welding of motorcycle headstock frame  
- **Robot Configuration**: 6-DOF articulated robot with 5 revolute joints  
- **Workspace Dimensions**: 2.75m × 2.25m × 0.1m (X, Y, Z)

---

## 🗂️ Repository Structure

| Folder | Description |
|--------|-------------|
| `Problem_Statements/` | Project instructions from Step 1 to Step 4 |
| `Implementation/` | Phase-wise submissions with plots and discussion |
| `MATLAB/` | All MATLAB code for kinematics, dynamics, and control |
| `Cad model/` | STEP/STL geometry for workspace and robot |
| `Project_Report.pdf` | Final report summarizing methodology and results |
| `README.md` | This file |

---

## 🚀 Key Features

- Forward & Inverse Kinematics using DH Parameters
- Path Planning using Potential Fields
- Jacobian-based Differential Kinematics
- Dynamic Modeling with Euler-Lagrange Equations
- Inverse Dynamics Control for Trajectory Tracking
- Obstacle avoidance with smooth path optimization
- Simulation using `ode45` for torque and trajectory validation

---

## 📸 CAD & Simulation

- Includes STL/STEP files for spatial modeling
- Simulated weld sequence execution with obstacle interaction
- Path smoothing, velocity control, and torque estimation

---

## 🧠 Future Extensions

- Integrate Reinforcement Learning for adaptive planning
- Convert symbolic dynamics to C for embedded control
- Add ROS/Gazebo support for hardware-in-the-loop testing
- Create a Digital Twin for real-time feedback and diagnostics

---

## 📄 License

This project is shared for academic and demonstration purposes. Please contact the author for any reuse or extension.

---

