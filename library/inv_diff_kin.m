function [q_dot_traj,q_ddot_traj] = inv_diff_kin(q,dp,ddp)
    J = get_J(q(1),q(2),q(3));
    q_dot_traj = pinv(J)*dp;
    J_dot = get_J_dot(q(1),q(2),q(3),q_dot_traj(1),q_dot_traj(2),q_dot_traj(3));
    q_ddot_traj = pinv(J)*(ddp-J_dot*q_dot_traj);
end
