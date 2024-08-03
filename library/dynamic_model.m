function dydt = dynamic_model(t, y, M_hat, C_hat, G_hat, tau)
    % Extract joint positions and velocities from the state vector
    q = y(1:3);       % Joint positions: q1, q2, q3
    q_dot = y(4:6);   % Joint velocities: q1_dot, q2_dot, q3_dot
    
    % Ensure M_hat, C_hat, G_hat, and tau are consistent in dimensions
    assert(all(size(q) == [3, 1]), 'q must be a 3x1 vector');
    assert(all(size(q_dot) == [3, 1]), 'q_dot must be a 3x1 vector');
    assert(all(size(M_hat) == [3, 3]), 'M_hat must be a 3x3 matrix');
    assert(all(size(C_hat) == [3, 1]), 'C_hat must be a 3x1 vector');
    assert(all(size(G_hat) == [3, 1]), 'G_hat must be a 3x1 vector');
    % assert(all(size(tau_f) == [3, 1]), 'tau_f_hat must be a 3x1 vector');
    assert(all(size(tau) == [3, 1]), 'tau must be a 3x1 vector');

    % Compute joint accelerations
    q_ddot = M_hat \ (tau - C_hat .* q_dot - G_hat);
    
    % Return the state derivatives
    dydt = [q_dot; q_ddot];
end