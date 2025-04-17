% --- Verification of Inverse Kinematics Solution (Independent Script) ---

% Define the target transformation matrix T_target (from the inverse kinematics problem)
T_target = [
   -0.8660, -0.5000, -0.0000, 648.8576;
   -0.2500,  0.4330, -0.8660, -13.2739;
    0.4330, -0.7500, -0.5000, 593.4050;
         0,       0,       0,      1.0000
];

% Solution obtained from inverse kinematics (q_solution)
% These values should match the output from InverseKinematics.m
q_solution = [-0.0000, 0.7854, -0.7854, 1.5708, -0.5236, 1.0472];

% Calculate the forward kinematics for the solution to get T_check
T_check = ForwardKinematicsVerification(q_solution);

% Display the target and calculated transformation matrices
disp('Target Transformation Matrix (T_target):');
disp(T_target);
disp('Calculated Transformation Matrix from q_solution (T_check):');
disp(T_check);

% Calculate position and orientation differences
position_difference = abs(T_target(1:3,4) - T_check(1:3,4));
orientation_difference = abs(T_target(1:3,1:3) - T_check(1:3,1:3));

% --- Local function for Forward Kinematics Calculation (Independent) ---
function T = ForwardKinematicsVerification(q)
    % Define DH parameters for each joint based on q_solution values
    dh_params = [
        0,       0,         194.95, q(1);
        310.77,  0,         0,      q(2);
        800.51,  pi/2,      0,      q(3);
        532.99,  -pi/2,     0,      q(4);
        0,       pi/2,      371.40, q(5);
        0,       0,         269.07, q(6)
    ];
    
    % Initialize transformation matrix as identity
    T = eye(4);

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
        T = T * Ti;
    end
end