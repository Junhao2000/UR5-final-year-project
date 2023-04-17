function [A, B] = num_linearize_dynamics(q, qd, tau)
    % compute numerical nonlinear dynamics equations
    m = @(q) M(q);
    c = @(q, qd) C(q, qd);
    G = @(q) g(q);
    
    % define state vector x
    x = [q; qd];

    % define vector-valued function
    f = @(x, M, C, g) [x(7:12); M(x(1:6)) \ (tau - C(x(1:6), x(7:12)) * x(7:12) - g(x(1:6)))];

    % define numerical values of the independent variable
    x_val = x;
    % define differentiation step size
    h = 1e-6;
    % compute Jacobian matrix A
    A = numerical_jacobian(@(x) f(x, m, c, G), x_val, h);
    
    % define vector-valued function
    G = @(tau_in) [qd; m(q) \ (tau_in - c(q, qd) * qd - G(q))];

    % compute Jacobian matrix B
    B = numerical_jacobian(G, tau, h);
end