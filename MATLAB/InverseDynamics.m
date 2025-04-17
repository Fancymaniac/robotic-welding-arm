%% Step 1: Define System Parameters
clear; clc;

% Masses, link lengths, and inertia values
m = [7.65, 2.01, 8.65, 4.03, 1.50, 1.82];  % Masses (kg)
L = [0.19495, 0.31077, 0.80051, 0.53299, 0.3714, 0.26907];  % Link lengths (m)
g = 9.81;  % Gravity

% Initial conditions
q0 = [0; 0; 0; 0; 0; 0];    % Initial positions (radians)
dq0 = [0; 0; 0; 0; 0; 0];   % Initial velocities

%% Step 2: Define Desired Trajectory (q_d, dq_d, ddq_d)
t_span = 0:0.01:5;  % Time span for simulation
q_d = @(t) [sin(t); sin(2*t); cos(0.5*t); sin(1.5*t); cos(t); sin(3*t)];  % Desired position
dq_d = @(t) [cos(t); 2*cos(2*t); -0.5*sin(0.5*t); 1.5*cos(1.5*t); -sin(t); 3*cos(3*t)]; % Desired velocity
ddq_d = @(t) [-sin(t); -4*sin(2*t); -0.25*cos(0.5*t); -2.25*sin(1.5*t); -cos(t); -9*sin(3*t)]; % Desired acceleration

%% Step 3: Define Dynamics (D, C, G) Symbolically
% Define symbolic variables
syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real

q = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

% Simplified Inertia Matrix D (Diagonal for simplicity)
D = diag(m .* L.^2 / 3);

% Coriolis and Centrifugal Matrix C
C = sym(zeros(6, 6));
for k = 1:6
    for i = 1:6
        for j = 1:6
            C(k, i) = C(k, i) + 0.5 * (diff(D(k, i), q(j)) + diff(D(k, j), q(i)) - diff(D(i, j), q(k))) * dq(j);
        end
    end
end

% Gravitational Forces G
G = m' .* g .* L' .* cos(q);

%% Step 4: Numerical Dynamics Functions
% Substitute values for masses and lengths into symbolic matrices
D_num = subs(D);
C_num = subs(C);
G_num = subs(G);

% Convert symbolic expressions to MATLAB functions
D_func = matlabFunction(D_num, 'Vars', {q});
C_func = matlabFunction(C_num, 'Vars', {q, dq});
G_func = matlabFunction(G_num, 'Vars', {q});

%% Step 5: Define Inverse Dynamics Control Law
% Control Law: tau = D(q)*ddq_d + C(q, dq)*dq_d + G(q)
tau_control = @(q, dq, t) D_func(q) * ddq_d(t) + C_func(q, dq) * dq_d(t) + G_func(q);

%% Step 6: Simulate the Dynamics using ODE45
% Define system dynamics
dynamics = @(t, y) [
    y(7:12);  % Velocities (dq)
    D_func(y(1:6)) \ (tau_control(y(1:6), y(7:12), t) - C_func(y(1:6), y(7:12)) * y(7:12) - G_func(y(1:6)))
];

% Initial state: [q; dq]
y0 = [q0; dq0];

% Solve the dynamics
[t, y] = ode45(dynamics, t_span, y0);

% Extract joint positions and velocities
q_sim = y(:, 1:6);
dq_sim = y(:, 7:12);

%% Step 7: Plot Results
figure;

% Plot Joint Positions
subplot(2, 1, 1);
plot(t, q_sim, 'LineWidth', 1.5);
title('Joint Positions with Inverse Dynamics Control');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
grid on;

% Plot Joint Velocities
subplot(2, 1, 2);
plot(t, dq_sim, 'LineWidth', 1.5);
title('Joint Velocities with Inverse Dynamics Control');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6');
grid on;