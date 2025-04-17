function T =ForwardKinematics(q)
    % DH parameters for each joint
    dh_params = [
        0,       0,         194.95, q(1);
        310.77,  0,         0,      q(2);
        800.51,  pi/2,      0,      q(3);
        532.99,  -pi/2,     0,      q(4);
        0,       pi/2,      371.40, q(5);
        0,       0,         269.07, q(6)
    ];
    
    % Initialize the transformation matrix as identity
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

%q1 = [0, pi/4, -pi/4, pi/2, -pi/6, pi/3]; 
%q2 = [pi/6, -pi/3, pi/4, -pi/2, pi/6, -pi/4];
q3 = [-pi/6, pi/3, -pi/4, pi/4, -pi/3, pi/6];

%disp('Forward Kinematics for q1:');
%disp(ForwardKinematics(q1));

%disp('Forward Kinematics for q2:');
%disp(ForwardKinematics(q2));

disp('Forward Kinematics for q3:');
disp(ForwardKinematics(q3));