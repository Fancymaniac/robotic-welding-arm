% --- Independent Inverse Kinematics Script ---

% Define the new target transformation matrix T_target based on q3
T_target = [
   -0.2399, -0.6502, -0.7209,  958.7592;
   -0.7122,  0.6225, -0.3245, -168.6802;
    0.6597,  0.4356, -0.6124,  669.6793;
         0,       0,       0,      1.0000
];

% Initial guess for joint angles
q_initial = [0, pi/4, -pi/4, pi/2, -pi/6, pi/3];

% Set optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 2000, 'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-8, 'FunctionTolerance', 1e-8);

% Define joint limits (optional)
q_min = [-pi, -pi/2, -pi, -pi, -pi/2, -pi];
q_max = [pi, pi/2, pi, pi, pi/2, pi];

% Objective function to minimize the pose error
objective = @(q) poseError(q, T_target);

% Run optimization to solve for joint angles that reach T_target
[q_solution, fval] = fmincon(objective, q_initial, [], [], [], [], q_min, q_max, [], options);

% Display the solution
disp('Inverse Kinematics Joint Angles Solution:');
disp(q_solution);

% --- Local function for Pose Error Calculation ---
function error = poseError(q, T_target)
    % Define DH parameters for each joint
    dh_params = [
        0,       0,         194.95, q(1);
        310.77,  0,         0,      q(2);
        800.51,  pi/2,      0,      q(3);
        532.99,  -pi/2,     0,      q(4);
        0,       pi/2,      371.40, q(5);
        0,       0,         269.07, q(6)
    ];
    
    % Initialize the transformation matrix as identity
    T_actual = eye(4);

    % Loop through each joint to calculate the total transformation matrix
    for i = 1:size(dh_params, 1)
        a = dh_params(i, 1);
        alpha = dh_params(i, 2);
        d = dh_params(i, 3);
        theta = dh_params(i, 4);
        
        % Compute individual transformation matrix using DH parameters
        Ti = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,           sin(alpha),            cos(alpha),            d;
              0,           0,                     0,                     1];
        
        % Update the total transformation matrix
        T_actual = T_actual * Ti;
    end

    % Compute position error as Euclidean distance
    position_error = norm(T_target(1:3,4) - T_actual(1:3,4));

    % Compute orientation error using Frobenius norm for rotation matrix difference
    orientation_error = norm(T_target(1:3,1:3) - T_actual(1:3,1:3), 'fro');

    % Total error as the sum of position and orientation errors
    error = position_error + orientation_error;
end
