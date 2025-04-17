% --- Jacobian Calculation for q3 ---

% Define joint angles for q3
q3 = [-pi/6, pi/3, -pi/4, pi/4, -pi/3, pi/6];

% Calculate the Jacobian matrix for q3
J = Jacob(q3);

% Display the Jacobian matrix for q3
disp('Jacobian Matrix for q3:');
disp(J);

% --- Local function to compute the Jacobian matrix ---
function J = Jacob(q)
    % Number of joints
    num_joints = length(q);

    % DH parameters
    dh_params = [
        0,       0,         194.95, q(1);
        310.77,  0,         0,      q(2);
        800.51,  pi/2,      0,      q(3);
        532.99,  -pi/2,     0,      q(4);
        0,       pi/2,      371.40, q(5);
        0,       0,         269.07, q(6)
    ];
    
    % Initialize transformation matrices and Jacobian
    T = eye(4); % Initial transformation matrix as identity
    J = zeros(6, num_joints); % Jacobian matrix (6x6 for 6 DOF)

    % Calculate the transformation matrix up to each joint and the Jacobian
    for i = 1:num_joints
        a = dh_params(i, 1);
        alpha = dh_params(i, 2);
        d = dh_params(i, 3);
        theta = dh_params(i, 4);
        
        % Compute the transformation matrix from i-1 to i
        Ti = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,           sin(alpha),            cos(alpha),            d;
              0,           0,                     0,                     1];
        
        % Update the transformation matrix from base to current joint i
        T = T * Ti;
        
        % Extract the position vector and z-axis for the current joint
        p = T(1:3, 4);
        z = T(1:3, 3);
        
        % Compute linear velocity part (position cross z-axis)
        Jv = cross(z, p);
        
        % Fill the Jacobian matrix (linear and angular parts)
        J(1:3, i) = Jv;  % Linear part
        J(4:6, i) = z;   % Angular part
    end
end