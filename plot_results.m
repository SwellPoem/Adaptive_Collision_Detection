function plot_results(t, r_hat_history, X_est_history, tau_history, tau_ext_history, q_traj, q_curr_history, ...
                      q_dot_traj, q_dot_curr_history, q_ddot_traj, q_ddot_curr_history, ...
                      up_threshold, down_threshold, f_ext_start, f_ext2_start)
    figure;
    plot(t(1:end), r_hat_history);
    title("Momentum Residual","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$kg * m/s$","Interpreter","Latex");
    axis([min(t) max(t) -50 50]);
    yline(up_threshold, "--r");
    yline(down_threshold, "--r");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;
    legend("$r_{1}$","$r_{2}$","$r_{3}$", "up threshold", "down threshold", "collision time", "Interpreter","Latex");

    % figure;
    % plot(t(1:end-1), X_est_history);
    % title("$ X_{est} $","Interpreter","Latex");
    % xlabel("$ s $", "Interpreter","Latex");
    % ylabel("$ X_{est} $","Interpreter","Latex");
    % xline(f_ext_start, "--b");
    % xline(f_ext2_start, "--b");
    % grid on;

    figure;
    plot(t(1:end), tau_ext_history);
    title("external torques","Interpreter","Latex")
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ Nm $","Interpreter","Latex");
    legend("$\tau_{ext,1}$","$\tau_{ext,2}$","$\tau_{ext,3}$", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    figure;
    plot(t(1:end), tau_history);
    title("joint torques","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ Nm $","Interpreter","Latex");
    legend("$\tau_{1}$","$\tau_{2}$","$\tau_{3}$", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    % Plot joint positions
    figure;
    subplot(3, 1, 1);
    plot(t, q_traj(1, :), "r", t, q_curr_history(1, :), "b");
    title("Joint 1 Position", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    subplot(3, 1, 2);
    plot(t, q_traj(2, :), "r", t, q_curr_history(2, :), "b");
    title("Joint 2 Position", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    subplot(3, 1, 3);
    plot(t, q_traj(3, :), "r", t, q_curr_history(3, :), "b");
    title("Joint 3 Position", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    % Plot joint velocities
    figure;
    subplot(3, 1, 1);
    plot(t, q_dot_traj(1, :), "r", t, q_dot_curr_history(1, :), "b");
    title("Joint 1 Velocity", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    subplot(3, 1, 2);
    plot(t, q_dot_traj(2, :), "r", t, q_dot_curr_history(2, :), "b");
    title("Joint 2 Velocity", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    subplot(3, 1, 3);
    plot(t, q_dot_traj(3, :), "r", t, q_dot_curr_history(3, :), "b");
    title("Joint 3 Velocity", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    % Plot joint accelerations
    figure;
    subplot(3, 1, 1);
    plot(t, q_ddot_traj(1, :), "r", t, q_ddot_curr_history(1, :), "b");
    title("Joint 1 Acceleration", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s^2 $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    subplot(3, 1, 2);
    plot(t, q_ddot_traj(2, :), "r", t, q_ddot_curr_history(2, :), "b");
    title("Joint 2 Acceleration", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s^2 $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

    subplot(3, 1, 3);
    plot(t, q_ddot_traj(3, :), "r", t, q_ddot_curr_history(3, :), "b");
    title("Joint 3 Acceleration", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s^2 $", "Interpreter","Latex");
    legend("Desired", "Actual", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;

end