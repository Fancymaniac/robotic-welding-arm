%% Step 1: Define System Parameters
clear; clc;

% Masses, link lengths, and gravity
m = [7.65, 2.01, 8.65, 4.03, 1.50, 1.82];  % Masses (kg)
L = [0.19495, 0.31077, 0.80051, 0.53299, 0.3714, 0.26907];  % Link lengths (m)
g = 9.81;  % Gravity (m/s^2)

% Inertia (simplified as solid rods about center)
I = (1/12) * m .* (L.^2);

% Initial conditions
q0 = [0; 0; 0; 0; 0; 0];    % Initial joint positions (radians)
dq0 = [0; 0; 0; 0; 0; 0];   % Initial joint velocities (rad/s)

%% Step 2: Define Desired Trajectory
t_span = 0:0.01:5;  % Time span for simulation

% Desired joint positions (q_d), velocities (dq_d), and accelerations (ddq_d)
q_d = @(t) [sin(t); sin(2*t); cos(0.5*t); sin(1.5*t); cos(t); sin(3*t)];
dq_d = @(t) [cos(t); 2*cos(2*t); -0.5*sin(0.5*t); 1.5*cos(1.5*t); -sin(t); 3*cos(3*t)];
ddq_d = @(t) [-sin(t); -4*sin(2*t); -0.25*cos(0.5*t); -2.25*sin(1.5*t); -cos(t); -9*sin(3*t)];

%% Step 3: Dynamics Matrices (D, C, G)
syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real

q = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

% Inertia Matrix D (diagonal approximation)
D = diag(m .* L.^2 / 3);

% Coriolis Matrix C (approximation with zeros)
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

% Convert to numerical functions
D_func = matlabFunction(subs(D), 'Vars', {q});
C_func = matlabFunction(subs(C), 'Vars', {q, dq});
G_func = matlabFunction(subs(G), 'Vars', {q});

%% Step 4: Inverse Dynamics Control Law
% Control law: tau = D(q)*ddq_d + C(q, dq)*dq_d + G(q)
tau_control = @(q, dq, t) D_func(q) * ddq_d(t) + C_func(q, dq) * dq_d(t) + G_func(q);

%% Step 5: Define System Dynamics
% State-space representation for ODE45
dynamics = @(t, y) [
    y(7:12);  % Velocities
    D_func(y(1:6)) \ (tau_control(y(1:6), y(7:12), t) - C_func(y(1:6), y(7:12)) * y(7:12) - G_func(y(1:6)))
];

% Initial state [q; dq]
y0 = [q0; dq0];

%% Step 6: Simulate the Dynamics
[t, y] = ode45(dynamics, t_span, y0);

% Extract joint positions, velocities, and compute torques
q_sim = y(:, 1:6);   % Joint positions
dq_sim = y(:, 7:12); % Joint velocities

tau_sim = zeros(length(t), 6);  % Torques
for i = 1:length(t)
    tau_sim(i, :) = tau_control(q_sim(i, :)', dq_sim(i, :)', t(i))';
end

%% Step 7: Plot Results
figure;

% Plot Joint Positions
subplot(3, 1, 1);
plot(t, q_sim, 'LineWidth', 1.5);
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
grid on;

% Plot Joint Velocities
subplot(3, 1, 2);
plot(t, dq_sim, 'LineWidth', 1.5);
title('Joint Velocities');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6');
grid on;

% Plot Joint Torques
subplot(3, 1, 3);
plot(t, tau_sim, 'LineWidth', 1.5);
title('Joint Torques');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('tau1', 'tau2', 'tau3', 'tau4', 'tau5', 'tau6');
grid on;