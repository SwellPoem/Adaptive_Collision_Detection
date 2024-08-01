function plot_results(t, r_hat_history, X_est_history, q_traj, q_curr_history, ...
                      q_dot_traj, q_dot_curr_history, q_ddot_traj, q_ddot_curr_history, ...
                      up_threshold, down_threshold, f_ext_start)
    figure;
    plot(t(1:end), r_hat_history);
    title('Momentum Residual (kg*m/s)');
    xlabel('Time (s)');
    ylabel('Residual');
    axis([min(t) max(t) -40 40]);
    legend('r1', 'r2', 'r3');
    yline(up_threshold, '--r');
    yline(down_threshold, '--r');
    xline(f_ext_start, '--b');

    figure;
    plot(t(1:end-1), X_est_history);
    title('X_est');
    xlabel('Time (s)');
    ylabel('X_est');
    xline(f_ext_start, '--b');

    % Plot joint positions
    figure;
    subplot(3, 1, 1);
    plot(t, q_traj(1, :), 'r', t, q_curr_history(1, :), 'b');
    title('Joint 1 Position');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    subplot(3, 1, 2);
    plot(t, q_traj(2, :), 'r', t, q_curr_history(2, :), 'b');
    title('Joint 2 Position');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    subplot(3, 1, 3);
    plot(t, q_traj(3, :), 'r', t, q_curr_history(3, :), 'b');
    title('Joint 3 Position');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    % Plot joint velocities
    figure;
    subplot(3, 1, 1);
    plot(t, q_dot_traj(1, :), 'r', t, q_dot_curr_history(1, :), 'b');
    title('Joint 1 Velocity');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    subplot(3, 1, 2);
    plot(t, q_dot_traj(2, :), 'r', t, q_dot_curr_history(2, :), 'b');
    title('Joint 2 Velocity');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    subplot(3, 1, 3);
    plot(t, q_dot_traj(3, :), 'r', t, q_dot_curr_history(3, :), 'b');
    title('Joint 3 Velocity');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    % Plot joint accelerations
    figure;
    subplot(3, 1, 1);
    plot(t, q_ddot_traj(1, :), 'r', t, q_ddot_curr_history(1, :), 'b');
    title('Joint 1 Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    subplot(3, 1, 2);
    plot(t, q_ddot_traj(2, :), 'r', t, q_ddot_curr_history(2, :), 'b');
    title('Joint 2 Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

    subplot(3, 1, 3);
    plot(t, q_ddot_traj(3, :), 'r', t, q_ddot_curr_history(3, :), 'b');
    title('Joint 3 Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend('Desired', 'Actual');
    xline(f_ext_start, '--b');

end