% function [q_trajectory, p_trajectory, dp_trajectory, ddp_trajectory] = excitation_trajectory(t, robot, q0)
%     % Define trajectory parameters
%     N = 3;  % Number of harmonics, increased for more complexity
%     omega = 0.2 * pi;  % Fundamental frequency
% 
%     % % Coefficients for Fourier series for each joint with more complexity
%     % a = [-0.658, 0.113, 0.574, -0.354, 0.231, -0.101;  % Joint 1
%     %       0.658, -0.494, 0.574, 0.374, -0.221, 0.105;  % Joint 2
%     %       0, 0.381, 0, -0.241, 0.131, -0.051];  % Joint 3
%     % b = [0.209, 0.124, -0.219, 0.189, -0.099, 0.044;  % Joint 1
%     %      0.611, -0.494, -0.574, 0.384, -0.211, 0.095;  % Joint 2
%     %      -0.477, 0.288, 0.455, -0.301, 0.177, -0.087];  % Joint 3
%     a = [-0.658, 0.658, 0;
%           0.113, -0.494, 0.381;
%           0.574, -0.574, 0];
%     b = [0, 0.611, -0.477;
%          0, -0.494, 0.288;
%          0, -0.574, 0.455];
% 
%     % 
%     %  a = [0.037, -0.457, -0.702;
%     %       0.731, 0.234, 1.316;
%     %       -0.768, 0.223, -0.614];
%     % b = [0.243, -0.457, -1.316;
%     %      0.289, 0.457, -0.632;
%     %      -0.274, -0.152, 0.860];
% 
%     % Initialize variables
%     q_trajectory = zeros(3, length(t));
%     p_trajectory = zeros(3, length(t));
%     dp_trajectory = zeros(3, length(t));
%     ddp_trajectory = zeros(3, length(t));
% 
%     % Compute trajectory
%     for i = 1:length(t)
%         for j = 1:N
%             q_trajectory(:, i) = q_trajectory(:, i) + (a(:, j) / (j * omega)) .* sin(j * omega * t(i)) - (b(:, j) / (j * omega)) .* cos(j * omega * t(i));
%             % q_dot_trajectory(:, i) = q_dot_trajectory(:, i) + a(:, j) .* cos(j * omega * t(i)) + b(:, j) .* sin(j * omega * t(i));
%             % q_ddot_trajectory(:, i) = q_ddot_trajectory(:, i) - a(:, j) .* sin(j * omega * t(i)) + b(:, j) .* cos(j * omega * t(i));
%         end
%         q_trajectory(:, i) = q_trajectory(:, i) + q0;
% 
%         % Convert joint positions to structure for forward kinematics
%         config = createConfigurationStruct(robot, q_trajectory(:, i));
% 
%         % Compute forward kinematics to get end-effector position
%         end_effector_position = getTransform(robot, config, 'body3', 'base');
% 
%         p_trajectory(:, i) = tform2trvec(end_effector_position)';
% 
%         % Compute velocities and accelerations using finite differences
%         if i > 1
%             dp_trajectory(:, i) = (p_trajectory(:, i) - p_trajectory(:, i-1)) / (t(i) - t(i-1));
%             if i > 2
%                 ddp_trajectory(:, i) = (dp_trajectory(:, i) - dp_trajectory(:, i-1)) / (t(i) - t(i-1));
%             end
%         end
%     end
% end

% function config = createConfigurationStruct(robot, q_array)
%     % Create a configuration structure from a numeric array of joint positions
%     num_joints = length(q_array);
%     config = homeConfiguration(robot);
%     for i = 1:num_joints
%         config(i).JointPosition = q_array(i);
%     end
% end



function [q_trajectory, p_trajectory, dp_trajectory, ddp_trajectory] = excitation_trajectory(t, robot, q0)
    % Convert initial configuration from struct to numeric vector
    q0_num = struct2array(q0);

    start_point = [0, 0.5, 0.7];
    radius = [0.6, 0.2, 0.2];

    % Ellipsoid parameters
    a = radius(1);  % Radius in x-direction
    b = radius(2);  % Radius in y-direction
    c = radius(3);  % Radius in z-direction
    T = max(t/2);  % Period of the trajectory
    omega = 2 * pi / T;  % Angular velocity

    % Center of the ellipsoid
    x_center = start_point(1);
    y_center = start_point(2);
    z_center = start_point(3);


    % Initialize variables
    num_joints = numel(q0_num);
    q_trajectory = zeros(num_joints, length(t));
    p_trajectory = zeros(3, length(t));
    dp_trajectory = zeros(3, length(t));
    ddp_trajectory = zeros(3, length(t));

    % Initial joint configuration
    q_trajectory(:, 1) = q0_num(:);

    % Inverse kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0, 0, 0, 1, 1, 1];
    initial_guess = q0;

    % Compute trajectory
    for i = 1:length(t)

        theta = omega * t(i);

        x = x_center + a * cos(2 * theta) * sin(theta);
        y = y_center + b * sin(2 * theta);
        z = z_center + c * cos(2 * theta);

        % x = x_center + a * cos(2 * theta);
        % y = y_center + b;
        % z = z_center + c * sin(theta);

        % Define the end-effector pose
        end_effector_pose = trvec2tform([x, y, z]);

        % Solve inverse kinematics to get joint angles
        [config, sol_info] = ik('body3', end_effector_pose, weights, initial_guess);

        % Update initial guess
        initial_guess = config;

        % Store joint angles
        q_trajectory(:, i) = struct2array(config);

        % Compute forward kinematics to get end-effector position
        end_effector_position = getTransform(robot, config, 'body3', 'base');
        p_trajectory(:, i) = tform2trvec(end_effector_position)';

        % Compute velocities and accelerations using finite differences
        if i > 1
            dp_trajectory(:, i) = (p_trajectory(:, i) - p_trajectory(:, i-1)) / (t(i) - t(i-1));
            if i > 2
                ddp_trajectory(:, i) = (dp_trajectory(:, i) - dp_trajectory(:, i-1)) / (t(i) - t(i-1));
            end
        end
    end
end

function q_array = struct2array(q_struct)
    num_joints = length(q_struct);
    q_array = zeros(num_joints, 1);
    for i = 1:num_joints
        q_array(i) = q_struct(i).JointPosition;
    end
end 