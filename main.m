clear variables
close all
clc

%% Load dynamic model parameters
load('data.mat');  % initial parameters

load('trajectory_data.mat'); %load trajectory to be displayed

%% Variables
syms q1 q2 q3 q1_dot q2_dot q3_dot q1_ddot q2_ddot q3_ddot real;

% symbolic coordinates, velocities, and accelerations
q_sym = [q1; q2; q3];
q_dot_sym = [q1_dot; q2_dot; q3_dot];
q_ddot_sym = [q1_ddot; q2_ddot; q3_ddot];

% torque initialization
tau = sym('tau', [3, 1]);

% lambda parameters for update
lambda1 = diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
lambda2 = diag([0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35]);

% redisual gain
K = diag([0.5, 0.5, 0.5]);

% controller gains
Kp = diag([80, 80, 80]);
Kd = diag([17, 17, 17]);

%% DH table
DHTABLE = [ pi/2 0 l(1) q(1);
             0 l(2) 0 q(2);
             0 l(3) 0 q(3)];

%% Creation of the robot to display
robot = createRobot(DHTABLE, l);

%% Robot dynamics
[M, C, G, M_par, C_par, G_par] = dynamics(m, I, l, dc, g0, q1, q2, q3);

% Dynamic equation
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

r_hat = zeros(3, 1);  % initial momentum residual
X_est = X;  % initial estimate of dynamic parameters
integral_term = zeros(3, 1);  % integral term for r_hat
prev_r_hat = r_hat;  % previous r_hat for derivative computation

% initialize the actual state
q_curr = q_traj(:, 1);
q_dot_curr = q_dot_traj(:, 1);

prev_q_dot_curr = q_dot_curr;

%% Preallocation
q_curr_history = zeros(size(q_traj));
q_dot_curr_history = zeros(size(q_traj));
q_ddot_curr_history = zeros(size(q_traj));
r_hat_history = zeros(size(q_traj));
tau_history = zeros(size(q_traj));
integrand_history = zeros(3, length(t)-1);
tau_ext_history = zeros(size(q_traj));

% store initial conditions
q_curr_history(:, 1) = q_curr;
q_dot_curr_history(:, 1) = q_dot_curr;

%% options
options = odeset('RelTol',1e-1,'AbsTol',1e-1);

up_threshold = 3.5;
down_threshold = -5;

%% external force parameters
f_ext = [0; 0; 12];  % Force in Newtons
f_ext_start = 6;  % Time when the force is applied
f_ext_duration = 0.2;  % Duration of the force application

%% figure initialization
figure;
hold on;
grid on;
axis("equal");
xlim([-1 1]);
ylim([-1 1]);
zlim([0 1.1]);
view(135, 5);
arrow=quiver3(0,0,0,0,0,0,'AutoScaleFactor',1/40,'LineWidth',2,'LineStyle', '-', 'Color',[0 0 1], ShowArrowHead='on');

% %% main loop
% for i = 1:length(t)-1
%     % update the joint angles
%     Q_0(1).JointPosition = q_curr(1);
%     Q_0(2).JointPosition = q_curr(2);
%     Q_0(3).JointPosition = q_curr(3);
% 
%     % desired state
%     q_des = q_traj(:, i);
%     q_dot_des = q_dot_traj(:, i);
%     q_ddot_des = q_ddot_traj(:, i);
% 
%     % inertia, Coriolis and gravity matrices of desired state
%     M = double(subs(M, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
%         {q_des(1), q_des(2), q_des(3), q_dot_des(1), q_dot_des(2), q_dot_des(3)}));
%     C = double(subs(C, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
%         {q_des(1), q_des(2), q_des(3), q_dot_des(1), q_dot_des(2), q_dot_des(3)}));
%     G = double(subs(G, {q1, q2, q3}, {q_des(1), q_des(2), q_des(3)}));
% 
%     % estimated inertia, Coriolis, and gravity matrices
%     M_hat = double(subs(M, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
%         {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3)}));
%     C_hat = double(subs(C, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
%         {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3)}));
%     G_hat = double(subs(G, {q1, q2, q3}, {q_curr(1), q_curr(2), q_curr(3)}));
% 
%     % compute control input tau
%     tau = M_hat * (q_ddot_des + Kp * (q_des - q_curr) + Kd * (q_dot_des - q_dot_curr)) + C_hat .* q_dot_curr + G_hat;
% 
%     % dynamics integration to update the actual state
%     [t_step, y_step] = ode45(@(t, y) dynamic_model(t, y, M_hat, C_hat, G_hat, tau), [t(i) t(i+1)], [q_curr; q_dot_curr], options);
% 
%     q_curr = y_step(end, 1:3).';
%     q_dot_curr = y_step(end, 4:6).';
% 
% 
%     % current acceleration
%     q_ddot_curr = (q_dot_curr - prev_q_dot_curr) / dt;
% 
% 
%      % external torque if the current time matches the external force time
%     if t(i) >= f_ext_start && t(i) <= (f_ext_start + f_ext_duration)
%         J_inv = get_J(q_curr(1), q_curr(2), q_curr(3))'; 
%         tau_ext = J_inv * f_ext; 
%     else
%         tau_ext = [0; 0; 0];
%     end
% 
%     % store actual values
%     q_curr_history(:, i) = q_curr;
%     q_dot_curr_history(:, i) = q_dot_curr;
%     q_ddot_curr_history(:, i) = q_ddot_curr;
% 
%     tau_history(:, i) = tau;
%     tau_ext_history(:, i) = tau_ext;
% 
%     % regressor matrix
%     Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
%         {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3), q_ddot_curr(1), q_ddot_curr(2), q_ddot_curr(3)}));
%     % Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
%     %     {q_traj(1), q_traj(2), q_traj(3), q_dot_traj(1), q_dot_traj(2), q_dot_traj(3), q_ddot_traj(1), q_ddot_traj(2), q_ddot_traj(3)}));
% 
%     % integrand
%     integrand = tau - Y_curr * X_est - r_hat;
% 
%     % trapezoidal rule
% 
%     % if i > 1
%     %     integral_term = integral_term + 0.5 * (integrand + prev_integrand) * dt;
%     % else
%     %     integral_term = integral_term + integrand * dt;
%     % end
%     integrand_history(:, i) = integrand;
% 
%     if i > 1
%         integral_term = trapz(t(1:i), integrand_history(:, 1:i), 2);  % Compute the integral
%     end
% 
%     % evolution of generalized momentum and its estimation
%     p_dot = C'*q_dot_curr - G + tau + tau_ext;
%     p_dot_hat = C_hat'*q_dot_curr - G_hat + tau + r_hat;
% 
%     % momentum residual
%     r_hat = K * integral_term;
% 
%     % update momentum residual
%     r_dot = K * (p_dot - p_dot_hat);
% 
%     % adaptive update of dynamic parameters
%     X_dot_hat = lambda1^-1 * Y_curr.' * r_hat + lambda2^-1 * Y_curr.' * r_dot;
%     X_est = X_est + X_dot_hat * dt;
% 
%     % Save values for plotting
%     r_hat_history(:, i) = r_hat;
%     X_est_history(:, i) = X_est;
% 
%     % save previous values for next iteration
%     prev_integrand = integrand;
%     prev_q_dot_curr = q_dot_curr;
% 
%     % Show
%     show(robot, Q_0, 'PreservePlot', false, 'FastUpdate', 1, 'Collisions', 'on');
%     % axis("equal");
%     % xlim([-1 1]);
%     % ylim([-1 1]);
%     % zlim([0 1]);
%     hold on;
%     plot3(p_traj(1, :), p_traj(2, :), p_traj(3, :), 'r-', 'LineWidth', 1);
% 
%     % actual end-effector position
%     end_effector_pos = get_end_eff_pos(q_curr(1), q_curr(2), q_curr(3));
% 
%     % plot the external force
%     if t(i) >= f_ext_start && t(i) <= (f_ext_start + f_ext_duration)
%         arrow.XData=end_effector_pos(1);
%         arrow.YData=end_effector_pos(2);
%         arrow.ZData=end_effector_pos(3);
%         arrow.UData=f_ext(1);
%         arrow.VData=f_ext(2);
%         arrow.WData=f_ext(3);
%     end
% 
%     drawnow;
% end

%% main loop
for i = 1:length(t)-1
    % update the joint angles
    Q_0(1).JointPosition = q_curr(1);
    Q_0(2).JointPosition = q_curr(2);
    Q_0(3).JointPosition = q_curr(3);

    % desired state
    q_des = q_traj(:, i);
    q_dot_des = q_dot_traj(:, i);
    q_ddot_des = q_ddot_traj(:, i);

    % inertia, Coriolis and gravity matrices of desired state
    M = double(subs(M, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_des(1), q_des(2), q_des(3), q_dot_des(1), q_dot_des(2), q_dot_des(3)}));
    C = double(subs(C, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_des(1), q_des(2), q_des(3), q_dot_des(1), q_dot_des(2), q_dot_des(3)}));
    G = double(subs(G, {q1, q2, q3}, {q_des(1), q_des(2), q_des(3)}));

    % estimated inertia, Coriolis, and gravity matrices
    M_hat = double(subs(M, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3)}));
    C_hat = double(subs(C, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, ...
        {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3)}));
    G_hat = double(subs(G, {q1, q2, q3}, {q_curr(1), q_curr(2), q_curr(3)}));

    % compute control input tau
    tau = M_hat * (q_ddot_des + Kp * (q_des - q_curr) + Kd * (q_dot_des - q_dot_curr)) + C_hat .* q_dot_curr + G_hat;

     % external torque if the current time matches the external force time
    if t(i) >= f_ext_start && t(i) <= (f_ext_start + f_ext_duration)
        J_inv = get_J(q_curr(1), q_curr(2), q_curr(3))'; 
        tau_ext = J_inv * f_ext;
        tau_new = tau + tau_ext;

        % dynamics integration to update the actual state
        [t_step, y_step] = ode45(@(t, y) dynamic_model(t, y, M_hat, C_hat, G_hat, tau_new), [t(i) t(i+1)], [q_curr; q_dot_curr], options);

        q_curr = y_step(end, 1:3).';
        q_dot_curr = y_step(end, 4:6).';


        % current acceleration
        q_ddot_curr = (q_dot_curr - prev_q_dot_curr) / dt;

        % store actual values
        q_curr_history(:, i) = q_curr;
        q_dot_curr_history(:, i) = q_dot_curr;
        q_ddot_curr_history(:, i) = q_ddot_curr;
    
        tau_history(:, i) = tau_new;
        tau_ext_history(:, i) = tau_ext;

        % regressor matrix
        Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
            {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3), q_ddot_curr(1), q_ddot_curr(2), q_ddot_curr(3)}));
        % Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
        %     {q_traj(1), q_traj(2), q_traj(3), q_dot_traj(1), q_dot_traj(2), q_dot_traj(3), q_ddot_traj(1), q_ddot_traj(2), q_ddot_traj(3)}));

        % integrand
        integrand = tau_new - Y_curr * X_est - r_hat;
    
        % trapezoidal rule

        % if i > 1
        %     integral_term = integral_term + 0.5 * (integrand + prev_integrand) * dt;
        % else
        %     integral_term = integral_term + integrand * dt;
        % end
        integrand_history(:, i) = integrand;

        if i > 1
            integral_term = trapz(t(1:i), integrand_history(:, 1:i), 2);  % Compute the integral
        end

        % evolution of generalized momentum and its estimation
        p_dot = C'*q_dot_curr - G + tau_new;
        p_dot_hat = C_hat'*q_dot_curr - G_hat + tau + r_hat;
    
        % momentum residual
        r_hat = K * integral_term;

        % update momentum residual
        r_dot = K * (p_dot - p_dot_hat);

        % adaptive update of dynamic parameters
        X_dot_hat = lambda1^-1 * Y_curr.' * r_hat + lambda2^-1 * Y_curr.' * r_dot;
        X_est = X_est + X_dot_hat * dt;

        % Save values for plotting
        r_hat_history(:, i) = r_hat;
        X_est_history(:, i) = X_est;

        % save previous values for next iteration
        prev_integrand = integrand;
        prev_q_dot_curr = q_dot_curr;

        % Show
        show(robot, Q_0, 'PreservePlot', false, 'FastUpdate', 1, 'Collisions', 'on');
        hold on;
        plot3(p_traj(1, :), p_traj(2, :), p_traj(3, :), 'r-', 'LineWidth', 1);
    
        % actual end-effector position
        end_effector_pos = get_end_eff_pos(q_curr(1), q_curr(2), q_curr(3));

        % plot the external force
        if t(i) >= f_ext_start && t(i) <= (f_ext_start + f_ext_duration)
            arrow.XData=end_effector_pos(1);
            arrow.YData=end_effector_pos(2);
            arrow.ZData=end_effector_pos(3);
            arrow.UData=f_ext(1);
            arrow.VData=f_ext(2);
            arrow.WData=f_ext(3);
        end
 
        drawnow;
    else
        tau_ext = [0; 0; 0];

        % dynamics integration to update the actual state
        [t_step, y_step] = ode45(@(t, y) dynamic_model(t, y, M_hat, C_hat, G_hat, tau), [t(i) t(i+1)], [q_curr; q_dot_curr], options);

        q_curr = y_step(end, 1:3).';
        q_dot_curr = y_step(end, 4:6).';


        % current acceleration
        q_ddot_curr = (q_dot_curr - prev_q_dot_curr) / dt;

        % store actual values
        q_curr_history(:, i) = q_curr;
        q_dot_curr_history(:, i) = q_dot_curr;
        q_ddot_curr_history(:, i) = q_ddot_curr;
    
        tau_history(:, i) = tau;
        tau_ext_history(:, i) = tau_ext;

        % regressor matrix
        Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
            {q_curr(1), q_curr(2), q_curr(3), q_dot_curr(1), q_dot_curr(2), q_dot_curr(3), q_ddot_curr(1), q_ddot_curr(2), q_ddot_curr(3)}));
        % Y_curr = double(subs(Y, {q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_ddot, q2_ddot, q3_ddot}, ...
        %     {q_traj(1), q_traj(2), q_traj(3), q_dot_traj(1), q_dot_traj(2), q_dot_traj(3), q_ddot_traj(1), q_ddot_traj(2), q_ddot_traj(3)}));

        % integrand
        integrand = tau - Y_curr * X_est - r_hat;
    
        % trapezoidal rule

        % if i > 1
        %     integral_term = integral_term + 0.5 * (integrand + prev_integrand) * dt;
        % else
        %     integral_term = integral_term + integrand * dt;
        % end
        integrand_history(:, i) = integrand;

        if i > 1
            integral_term = trapz(t(1:i), integrand_history(:, 1:i), 2);  % Compute the integral
        end

        % evolution of generalized momentum and its estimation
        p_dot = C'*q_dot_curr - G + tau + tau_ext;
        p_dot_hat = C_hat'*q_dot_curr - G_hat + tau + r_hat;
    
        % momentum residual
        r_hat = K * integral_term;

        % update momentum residual
        r_dot = K * (p_dot - p_dot_hat);

        % adaptive update of dynamic parameters
        X_dot_hat = lambda1^-1 * Y_curr.' * r_hat + lambda2^-1 * Y_curr.' * r_dot;
        X_est = X_est + X_dot_hat * dt;

        % Save values for plotting
        r_hat_history(:, i) = r_hat;
        X_est_history(:, i) = X_est;

        % save previous values for next iteration
        prev_integrand = integrand;
        prev_q_dot_curr = q_dot_curr;

        % Show
        show(robot, Q_0, 'PreservePlot', false, 'FastUpdate', 1, 'Collisions', 'on');
  
        hold on;
        plot3(p_traj(1, :), p_traj(2, :), p_traj(3, :), 'r-', 'LineWidth', 1);
    
        % actual end-effector position
        end_effector_pos = get_end_eff_pos(q_curr(1), q_curr(2), q_curr(3));

        arrow.XData=0;
        arrow.YData=0;
        arrow.ZData=0;
        arrow.UData=0;
        arrow.VData=0;
        arrow.WData=0;

        drawnow;
    end
end

%% plots

plot_results(t, r_hat_history, X_est_history, q_traj, q_curr_history, ...
                      q_dot_traj, q_dot_curr_history, q_ddot_traj, q_ddot_curr_history, ...
                      up_threshold, down_threshold, f_ext_start);