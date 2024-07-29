% Main script to launch simulations
clear variables
close all
clc

%% Load dynamic model parameters
load('data.mat');  % Contains initial parameters

load('trajectory_data.mat');

%% Variables
syms q1 q2 q3 q1_dot q2_dot q3_dot q1_ddot q2_ddot q3_ddot real;

% Symbolic coordinates, velocities, and accelerations
q_sym = [q1; q2; q3];
q_dot_sym = [q1_dot; q2_dot; q3_dot];
q_ddot_sym = [q1_ddot; q2_ddot; q3_ddot];

% Torque inputs
tau = sym('tau', [3, 1]);

% lambda1 = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]);
% lambda2 = diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]);
lambda1 = diag([5, 5, 5, 5, 5, 5, 5, 5]);
lambda2 = diag([2, 2, 2, 2, 2, 2, 2, 2]);

K = diag([0.5, 0.5, 0.5]);

% Controller gains
Kp = diag([30, 30, 30]);
Kd = diag([11, 11, 11]);

%% DH table
DHTABLE = [ pi/2 0 l(1) q(1);
             0 l(2) 0 q(2);
             0 l(3) 0 q(3)];

%% Creation of the robot to display
robot = createRobot(DHTABLE, l);

%% Robot dynamics
[M, C, G, M_par, C_par, G_par] = dynamics(m, I, l, dc, g0, q1, q2, q3);

% Dynamic equation with friction
dynamic_eq = M * q_ddot_sym + C + G == tau;

%% Regressor matrix
[Y, X] = regressor_matrix(M_par, C_par, G_par, m, I, l, dc, g0, q1_ddot, q2_ddot, q3_ddot);

%% Excitation trajectory
T = 20; % Total time
dt = 0.1; % Time step
t = 0:dt:T; % Time vector

Q_0 = homeConfiguration(robot);

[q_traj, p_traj, dp_traj, ddp_traj] = excitation_trajectory(t, robot, Q_0);
save('trajectory_data.mat', 'q_traj', 'p_traj', 'dp_traj', 'ddp_traj');

[q_dot_traj, q_ddot_traj] = inv_diff_kin(q_traj, dp_traj, ddp_traj);

initial_conditions = [q_traj(:,1); q_dot_traj(:,1)];
r_hat = zeros(3, 1);  % Initial momentum residual
X_est = X;  % Initial estimate of dynamic parameters
integral_term = zeros(3, 1);  % Initialize the integral term for r_hat
prev_r_hat = r_hat;  % Initialize previous r_hat for derivative computation

% Initialize the actual state
q_curr = q_traj(:, 1);
q_dot_curr = q_dot_traj(:, 1);

prev_q_dot_curr = q_dot_curr;

%% Initialize storage for actual values
q_curr_history = zeros(size(q_traj));
q_dot_curr_history = zeros(size(q_traj));
q_ddot_curr_history = zeros(size(q_traj));
r_hat_history = zeros(size(q_traj));
tau_history = zeros(size(q_traj));
integrand_history = zeros(3, length(t)-1);

% Store initial conditions
q_curr_history(:, 1) = q_curr;
q_dot_curr_history(:, 1) = q_dot_curr;

options = odeset('RelTol',1e-6,'AbsTol',1e-10);

up_threshold = 3;
down_threshold = -6;

%% main loop
for i = 1:length(t)-1
    % Update the joint angles
    Q_0(1).JointPosition = q_traj(1, i);
    Q_0(2).JointPosition = q_traj(2, i);
    Q_0(3).JointPosition = q_traj(3, i);

    % Desired state
    q_des = q_traj(:, i);
    q_dot_des = q_dot_traj(:, i);
    q_ddot_des = q_ddot_traj(:, i);


    M = double(subs(M, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_des(1), q_des(2), q_des(3), q_dot_des(1), q_dot_des(2), q_dot_des(3)}));
    C = double(subs(C, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_des(1), q_des(2), q_des(3), q_dot_des(1), q_dot_des(2), q_dot_des(3)}));
    G = double(subs(G, {q1, q2, q3}, {q_des(1), q_des(2), q_des(3)}));

    % Compute the estimated inertia, Coriolis, and gravity matrices
    M_hat = double(subs(M, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3)}));
    C_hat = double(subs(C, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3)}));
    G_hat = double(subs(G, {q1, q2, q3}, {q_curr(1), q_curr(2), q_curr(3)}));

    % Compute control input tau
    tau = M_hat * (q_ddot_des + Kp * (q_des - q_curr) + Kd * (q_dot_des - q_dot_curr)) + C_hat .* q_dot_curr + G_hat;

    % Integrate the dynamics to update the actual state
    [t_step, y_step] = ode45(@(t, y) dynamic_model(t, y, M_hat, C_hat, G_hat, tau), [t(i) t(i+1)], [q_curr; q_dot_curr], options);
    

    q_curr = y_step(end, 1:3).';
    q_dot_curr = y_step(end, 4:6).';


    % Compute current acceleration
    q_ddot_curr = (q_dot_curr - prev_q_dot_curr) / dt;

    % Store actual values
    q_curr_history(:, i) = q_curr;
    q_dot_curr_history(:, i) = q_dot_curr;
    q_ddot_curr_history(:, i) = q_ddot_curr;
    
    tau_history(:, i) = tau;

    % Compute the regressor matrix
    Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
        {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3), q_ddot_curr(1), q_ddot_curr(2), q_ddot_curr(3)}));
    % Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
    %     {q_traj(1), q_traj(2), q_traj(3), q_dot_traj(1), q_dot_traj(2), q_dot_traj(3), q_ddot_traj(1), q_ddot_traj(2), q_ddot_traj(3)}));

    % Calculate integrand
    integrand = tau - Y_curr * X_est - r_hat;
    
    % trapezoidal rule
    % if i > 1
    %     integral_term = (integral_term + 0.5 * (integrand + prev_integrand)) * dt;
    % else
    %     integral_term = (integral_term + integrand) * dt;
    % end
    integrand_history(:, i) = integrand;

    if i > 1
        integral_term = trapz(t(1:i), integrand_history(:, 1:i), 2);  % Compute the integral
    end

    %compute evolution of generalized momentum and its estimation
    p_dot = C'*q_dot_curr - G + tau;
    p_dot_hat = C_hat'*q_dot_curr - G_hat + tau + r_hat;
    
    % Compute momentum residual
    r_hat = K * integral_term;

    % Update momentum residual
    r_dot = K * (p_dot - p_dot_hat);

    % Adaptive update of dynamic parameters
    X_dot_hat = lambda1^-1 * Y_curr.' * r_hat + lambda2^-1 * Y_curr.' * r_dot;
    X_est = X_est + X_dot_hat * dt;

    % Save values for plotting
    r_hat_history(:, i) = r_hat;
    X_est_history(:, i) = X_est;

    % Save previous values for next iteration
    prev_integrand = integrand;
    prev_q_dot_curr = q_dot_curr;

    % Show the robot
    show(robot, Q_0, 'PreservePlot', false);
    % axis("equal");
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([0 1]);
    hold on;
    plot3(p_traj(1, :), p_traj(2, :), p_traj(3, :), 'r-', 'LineWidth', 1);
    drawnow;
end

%% plots
% plot_joints(t, q_traj, q_dot_traj, q_ddot_traj);

figure;
plot(t(1:end), r_hat_history);
title('Momentum Residual (r)');
xlabel('Time (s)');
ylabel('Residual');
axis([min(t) max(t) -40 40]);
legend('r1', 'r2', 'r3');
yline(up_threshold, '--r');
yline(down_threshold, '--r');

figure;
plot(t(1:end-1), X_est_history);
title('X_est');
xlabel('Time (s)');
ylabel('X_est');

% Plot joint positions
figure;
subplot(3, 1, 1);
plot(t, q_traj(1, :), 'r', t, q_curr_history(1, :), 'b');
title('Joint 1 Position');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Desired', 'Actual');

subplot(3, 1, 2);
plot(t, q_traj(2, :), 'r', t, q_curr_history(2, :), 'b');
title('Joint 2 Position');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Desired', 'Actual');

subplot(3, 1, 3);
plot(t, q_traj(3, :), 'r', t, q_curr_history(3, :), 'b');
title('Joint 3 Position');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Desired', 'Actual');

% Plot joint velocities
figure;
subplot(3, 1, 1);
plot(t, q_dot_traj(1, :), 'r', t, q_dot_curr_history(1, :), 'b');
title('Joint 1 Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Desired', 'Actual');

subplot(3, 1, 2);
plot(t, q_dot_traj(2, :), 'r', t, q_dot_curr_history(2, :), 'b');
title('Joint 2 Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Desired', 'Actual');

subplot(3, 1, 3);
plot(t, q_dot_traj(3, :), 'r', t, q_dot_curr_history(3, :), 'b');
title('Joint 3 Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Desired', 'Actual');

% Plot joint accelerations
figure;
subplot(3, 1, 1);
plot(t, q_ddot_traj(1, :), 'r', t, q_ddot_curr_history(1, :), 'b');
title('Joint 1 Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Desired', 'Actual');

subplot(3, 1, 2);
plot(t, q_ddot_traj(2, :), 'r', t, q_ddot_curr_history(2, :), 'b');
title('Joint 2 Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Desired', 'Actual');

subplot(3, 1, 3);
plot(t, q_ddot_traj(3, :), 'r', t, q_ddot_curr_history(3, :), 'b');
title('Joint 3 Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Desired', 'Actual');
