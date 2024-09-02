function plot_results(t, r_hat_history, X_est_history, tau_history, tau_ext_history, tau_f_history, f_ext_history, q_traj, q_curr_history, ...
                      q_dot_traj, q_dot_curr_history, q_ddot_traj, q_ddot_curr_history, ...
                      up_threshold, down_threshold, f_ext_start, f_ext2_start, p_traj, dp_traj, ddp_traj, end_effector_pos_history)

    figure;
    plot(t(1:end), r_hat_history, "LineWidth", 1);
    title("Momentum Residual","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$kg * m/s$","Interpreter","Latex");
    axis([min(t) max(t) -30 30]);
    yline(up_threshold, "--r");
    yline(down_threshold, "--r");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;
    legend("$r_{1}$","$r_{2}$","$r_{3}$", "up threshold", "down threshold", "collision", "Interpreter","Latex",  "Location", "south", "Orientation","horizontal");
    % exportgraphics(gcf,'r.png','Resolution',300, 'BackgroundColor','white')

    figure;
    plot(t(1:end-1), X_est_history, "LineWidth", 1);
    title("$ X_{est} $","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ X_{est} $","Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("$x_{1}$","$x_{2}$","$x_{3}$", "$x_{4}$", "$x_{5}$", "$x_{6}$", "$x_{7}$", "$x_{8}$","$x_{9}$", "$x_{10}$", "$x_{11}$", "Interpreter","Latex",  "Location", "eastoutside");
    grid on;
    % exportgraphics(gcf,'x_upd.png','Resolution',300, 'BackgroundColor','white')


    figure;
    subplot(2, 1, 1);
    plot(t(1:end), f_ext_history, "LineWidth", 1);
    title("external force","Interpreter","Latex")
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ N $","Interpreter","Latex");
    axis([min(t) max(t) -40 10]);
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("$f_{ext,x}$","$f_{ext,y}$","$f_{ext,z}$", "collision", "Interpreter","Latex", "Location", "south", "Orientation","horizontal");
    grid on;

    subplot(2, 1, 2);
    plot(t(1:end), tau_ext_history, "LineWidth", 1);
    title("external torques","Interpreter","Latex")
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ Nm $","Interpreter","Latex");
    axis([min(t) max(t) -10 20]);
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("$\tau_{ext,1}$","$\tau_{ext,2}$","$\tau_{ext,3}$", "collision", "Interpreter","Latex",  "Location", "north", "Orientation","horizontal");
    grid on;
    % exportgraphics(gcf,'ext_force_torque.png','Resolution',300, 'BackgroundColor','white')

    
    figure;
    plot(t(1:end), tau_history, "LineWidth", 1);
    title("joint torques","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ Nm $","Interpreter","Latex");
    legend("$\tau_{1}$","$\tau_{2}$","$\tau_{3}$", "Interpreter","Latex", "Location", "south", "Orientation","horizontal");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    grid on;
    % exportgraphics(gcf,'tau.png','Resolution',300, 'BackgroundColor','white')


    % figure;
    % plot(t(1:end), tau_f_history);
    % title("friction","Interpreter","Latex");
    % xlabel("$ s $", "Interpreter","Latex");
    % ylabel("$ N $","Interpreter","Latex");
    % legend("$\tau_f{1}$","$\tau_f{2}$","$\tau_f{3}$", "Interpreter","Latex");
    % xline(f_ext_start, "--b");
    % xline(f_ext2_start, "--b");
    % grid on;

    %plot end-effector actual position, velocity and acceleration
    figure;
    subplot(3, 1, 1);
    plot(t(1:end), p_traj, "LineWidth", 1);
    xlabel("$ t [s] $", "Interpreter","Latex");
    ylabel("$ p_{e, des} [m] $","Interpreter","Latex");
    legend("$p_{e,1}$","$p_{e,2}$","$p_{e,3}$", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 2);
    plot(t(1:end), dp_traj, "LineWidth", 1);
    xlabel("$ t [s] $", "Interpreter","Latex");
    ylabel("$ \dot{p}_{e, des} [m/s] $","Interpreter","Latex");
    legend("$\dot{p}_{e,1}$","$\dot{p}_{e,2}$","$\dot{p}_{e,3}$", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 3);
    plot(t(1:end), ddp_traj, "LineWidth", 1);
    xlabel("$ t [s] $", "Interpreter","Latex");
    ylabel("$ \ddot{p}_{e, des} [m/s^2] $","Interpreter","Latex");
    legend("$\ddot{p}_{e,1}$","$\ddot{p}_{e,2}$","$\ddot{p}_{e,3}$", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;
    % exportgraphics(gcf,'ee_pos_vel_acc.png','Resolution',300, 'BackgroundColor','white')



    %plot end-effector desired pos vs actual pos
    figure;
    subplot(3, 1, 1);
    plot(t(1:end), p_traj(1,:), "r", t(1:end), end_effector_pos_history(1,:), "b", "LineWidth", 1);
    title(" end effector x position ","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ m $","Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual","collision", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    
    grid on;

    subplot(3, 1, 2);
    plot(t(1:end), p_traj(2,:), "r", t(1:end), end_effector_pos_history(2,:), "b", "LineWidth", 1);
    title(" end effector y position ","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ m $","Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual","collision", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 3);
    plot(t(1:end), p_traj(3,:), "r", t(1:end), end_effector_pos_history(3,:), "b", "LineWidth", 1);
    title(" end effector z position ","Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ m $","Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision","Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;
    % exportgraphics(gcf,'pos.png','Resolution',300, 'BackgroundColor','white')


    % Plot joint positions
    figure;
    subplot(3, 1, 1);
    plot(t, q_traj(1, :), "r", t, q_curr_history(1, :), "b", "LineWidth", 1);
    title("Joint 1 Position", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision","Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 2);
    plot(t, q_traj(2, :), "r", t, q_curr_history(2, :), "b", "LineWidth", 1);
    title("Joint 2 Position", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision","Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 3);
    plot(t, q_traj(3, :), "r", t, q_curr_history(3, :), "b", "LineWidth", 1);
    title("Joint 3 Position", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual","collision", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;
    % exportgraphics(gcf,'q.png','Resolution',300, 'BackgroundColor','white')


    % Plot joint velocities
    figure;
    subplot(3, 1, 1);
    plot(t, q_dot_traj(1, :), "r", t, q_dot_curr_history(1, :), "b", "LineWidth", 1);
    title("Joint 1 Velocity", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 2);
    plot(t, q_dot_traj(2, :), "r", t, q_dot_curr_history(2, :), "b", "LineWidth", 1);
    title("Joint 2 Velocity", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");

    grid on;

    subplot(3, 1, 3);
    plot(t, q_dot_traj(3, :), "r", t, q_dot_curr_history(3, :), "b", "LineWidth", 1);
    title("Joint 3 Velocity", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision","Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;
    % exportgraphics(gcf,'vel.png','Resolution',300, 'BackgroundColor','white')


    % Plot joint accelerations
    figure;
    subplot(3, 1, 1);
    plot(t, q_ddot_traj(1, :), "r", t, q_ddot_curr_history(1, :), "b", "LineWidth", 1);
    title("Joint 1 Acceleration", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s^2 $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision", "Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");

    grid on;

    subplot(3, 1, 2);
    plot(t, q_ddot_traj(2, :), "r", t, q_ddot_curr_history(2, :), "b", "LineWidth", 1);
    title("Joint 2 Acceleration", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s^2 $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision","Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;

    subplot(3, 1, 3);
    plot(t, q_ddot_traj(3, :), "r", t, q_ddot_curr_history(3, :), "b", "LineWidth", 1);
    title("Joint 3 Acceleration", "Interpreter","Latex");
    xlabel("$ s $", "Interpreter","Latex");
    ylabel("$ rad/s^2 $", "Interpreter","Latex");
    xline(f_ext_start, "--b");
    xline(f_ext2_start, "--b");
    legend("Desired", "Actual", "collision","Interpreter","Latex", "Location", "northoutside", "Orientation","horizontal");
    grid on;
    % exportgraphics(gcf,'acc.png','Resolution',300, 'BackgroundColor','white')


end