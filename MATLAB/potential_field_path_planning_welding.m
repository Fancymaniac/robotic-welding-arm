clc;
function potential_field_path_planning_welding()
    % Define constants
    k_att = 5.0;         % Attractive constant to significantly strengthen goal pull
    k_rep = 40.0;        % Repulsive constant to weaken obstacle avoidance
    d0 = 50;             % Decreased influence distance for repulsive field (in millimeters)

    % Define goal position (Weld Point 3)
    goal = [1330.01, 0, 190.32]; % End position in millimeters

    % Define obstacles as a set of points (in millimeters)
    obstacles = [
        1250.01, 0, 190.32;    % Middle Weld Point
        1250.01, -21.35, 190.32; % Left Side Weld Point
        1250.01, 21.35, 190.32   % Right Side Weld Point
    ];

    % Initial position (Weld Point 1)
    current_pos = [1170.01, 0, 190.32]; % Start position in millimeters

    % Define step size for movement (in millimeters)
    step_size = 30; % Increased step size for faster movement

    % Set tolerance for reaching the goal (in millimeters)
    tolerance = 15; % Slightly reduced tolerance for more precise stopping

    % Plot the environment
    figure;
    hold on;
    plot3(goal(1), goal(2), goal(3), 'r*', 'MarkerSize', 10); % Goal position
    plot3(obstacles(:, 1), obstacles(:, 2), obstacles(:, 3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Obstacles
    plot3(current_pos(1), current_pos(2), current_pos(3), 'bo'); % Starting position
    axis equal;
    grid on;
    xlabel('X-axis (millimeters)');
    ylabel('Y-axis (millimeters)');
    zlabel('Z-axis (millimeters)');
    title('Potential Field Path Planning for Welding Points');

    % Iteratively move towards the goal
    iteration = 0; % to avoid infinite loop
    max_iterations = 2000; % Allow enough iterations for goal completion

    while norm(current_pos - goal) > tolerance && iteration < max_iterations
        % Calculate attractive force (towards goal)
        F_att = -k_att * (current_pos - goal);

        % Calculate repulsive force (from obstacles)
        F_rep = [0, 0, 0];
        for i = 1:size(obstacles, 1)
            d_obs = norm(current_pos - obstacles(i, :));
            if d_obs < d0
                % Repulsive force calculation with reduced penalty for closer distances
                penalty = (d0 / d_obs); % Reduced penalty to allow smoother movement
                F_rep = F_rep + penalty * k_rep * (1/d_obs - 1/d0) * (1/d_obs^2) * ...
                        (current_pos - obstacles(i, :)) / d_obs;
            end
        end

        % Total force (combined attractive and repulsive)
        F_total = F_att + F_rep;

        % Normalize the total force and apply step size to update position
        if norm(F_total) > 0
            % Adaptive step size based on force balance
            adaptive_step_size = step_size / (1 + norm(F_rep));
            % Adding a small random perturbation to escape local minima
            random_perturbation = (rand(1, 3) - 0.5) * 1.0; % Increased random noise for better escape
            current_pos = current_pos + adaptive_step_size * F_total / norm(F_total) + random_perturbation;
        end

        % Plotting the new position at every iteration for a more elaborate path
        plot3(current_pos(1), current_pos(2), current_pos(3), 'bo');
        
        iteration = iteration + 1;
    end

    % Plot the final position
    plot3(current_pos(1), current_pos(2), current_pos(3), 'g*', 'MarkerSize', 10); % Final position
    legend('Goal (End Weld Point)', 'Obstacles (Weld Points)', 'Robot Path', 'Final Position');
    hold off;

    if iteration >= max_iterations
        disp('Reached maximum number of iterations. Path might be stuck.');
    end
end
potential_field_path_planning_welding();