%% Step 1: Define symbolic variables for generalized coordinates and velocities
clear;
clc;

syms q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6 real  % Joint angles and velocities
syms g real  % Gravitational constant
syms t real  % Time variable

% Link Parameters
m = [7.65, 2.01, 8.65, 4.03, 1.50, 1.82];  % Masses (kg)
L = [0.19495, 0.31077, 0.80051, 0.53299, 0.3714, 0.26907];  % Link lengths (m)
h = [L(1)/2, L(2)/2, L(3)/2, L(4)/2, L(5)/2, L(6)/2];  % Heights of the center of mass (midpoint of each link)
g = 9.81;  % Gravitational constant in m/s^2

% Inertia of each link (solid rod assumption, rotated about the center)
I = (1/12) * m .* (L.^2);

% Generalized coordinates and velocities
q = [q1, q2, q3, q4, q5, q6];  % Generalized coordinates (joint angles)
dq = [dq1, dq2, dq3, dq4, dq5, dq6];  % Generalized velocities (joint angular velocities)

%% Step 2: Calculate Rotation Matrices (Jacobian) for each link
% Initialize Jacobian matrices
Jv = sym(zeros(2, 6));  % Linear velocity Jacobian (2D planar case)
Jw = sym(zeros(1, 6));  % Angular velocity Jacobian

for i = 1:6
    % Linear velocity Jacobian
    Jv(:, i) = [sum(L(1:i)) * cos(sum(q(1:i))); sum(L(1:i)) * sin(sum(q(1:i)))];
    
    % Angular velocity Jacobian
    Jw(i) = 1;  % Each joint contributes to angular velocity in the 2D planar case
end

disp('Step 2: Jacobian Matrices (Linear and Angular Velocities):');
disp('Linear Velocity Jacobian (Jv):');
disp(Jv);
disp('Angular Velocity Jacobian (Jw):');
disp(Jw);

%% Step 3: Define Kinetic Energy (T) for the manipulator
T = 0;  % Initialize total kinetic energy

for i = 1:6
    % Translational kinetic energy (velocity at CoM)
    v_i = dq(i) * L(i) / 2;  % Approximation for velocity of the CoM
    T = T + (1/2) * m(i) * v_i^2;  % Translational kinetic energy
    
    % Rotational kinetic energy
    omega_i = dq(i);  % Angular velocity for link i
    T = T + (1/2) * I(i) * omega_i^2;  % Rotational kinetic energy
end

disp('Step 3: Total Kinetic Energy (T):');
disp(T);

%% Step 4: Define Potential Energy (V)
V = 0;  % Initialize potential energy

for i = 1:6
    V = V + m(i) * g * h(i) * cos(q(i));  % Gravitational potential energy for each link
end

disp('Step 4: Potential Energy (V):');
disp(V);

%% Step 5: Define Lagrangian and Dynamic Equations of Motion
Lagrangian = T - V;  % Lagrangian (T - V)

% Initialize array for Euler-Lagrange equations
tau = sym('tau', [1, 6]);  % Generalized torques (forces) for each joint
eqns = sym(zeros(6, 1));  % Array to hold the equations

for i = 1:6
    % Derivatives for Euler-Lagrange equation
    dL_dq_dot = diff(Lagrangian, dq(i));  % Derivative of L with respect to dq(i)
    dL_dq = diff(Lagrangian, q(i));  % Derivative of L with respect to q(i)
    
    % Time derivative of dL_dq_dot
    dL_dq_dot_dt = diff(dL_dq_dot, t);
    
    % Euler-Lagrange equation: d/dt(dL/dq_dot) - dL/dq = tau
    eqns(i) = dL_dq_dot_dt - dL_dq == tau(i);
end

disp('Step 5: Euler-Lagrange Equations (Dynamic Equations of Motion):');
disp(eqns);

%% Step 6: Export Equations for Numerical Use (Optional)
% Simplify and rearrange equations for numerical simulation (if needed)
simplified_eqns = simplify(eqns);
disp('Simplified Dynamic Equations:');
disp(simplified_eqns);