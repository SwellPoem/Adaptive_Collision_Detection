function plot_joints(t, q_trajectory, q_dot_trajectory, q_ddot_trajectory)
    figure;
    subplot(3,1,1);
    plot(t, q_trajectory);
    title('Joint Positions');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Joint 1', 'Joint 2', 'Joint 3');

    subplot(3,1,2);
    plot(t, q_dot_trajectory);
    title('Joint Velocities');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Joint 1', 'Joint 2', 'Joint 3');

    subplot(3,1,3);
    plot(t, q_ddot_trajectory);
    title('Joint Accelerations');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend('Joint 1', 'Joint 2', 'Joint 3');
end