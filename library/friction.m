function [tau_f, tau_f_par] = friction(fv, q_dot_sym)
    
    syms a9 a10 a11 real

    % Friction model: tau_f = fc * sign(q_dot) + fv * q_dot + fo
    % tau_f = fc .* sign(q_dot_sym) + fv .* q_dot_sym + fo;
    tau_f = fv .* q_dot_sym;

    %a9 = fv1
    %a10 = fv2
    %a11 = fv3
    tau_f_par(1,1)= subs(tau_f(1,1), fv(1), a9);
    tau_f_par(2,1)= subs(tau_f(2,1), fv(2), a10);
    tau_f_par(3,1)= subs(tau_f(3,1), fv(3), a11);
end

