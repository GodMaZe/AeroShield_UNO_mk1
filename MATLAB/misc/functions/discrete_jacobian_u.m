function J = discrete_jacobian_u(f, t, x, u, eps)
% Calculate the numerical finite-difference Jacobian of f(x)
arguments (Input)
    f;
    t;
    x;
    u;
    eps = 1e-3;
end
arguments (Output)
    J
end
    fx = f(t, x, u);
    m = numel(fx);
    r = numel(u);
    J = zeros(m, r);
    for i = 1:r
        u1 = u;
        u1(i) = u1(i) + eps; % (x + h)
        fx1 = f(t, x, u1); % f(x + h)
        % df/dx = \lim_{h -> 0} \frac{f(x + h) - f(x)}{h}
        J(:, i) = (fx1 - fx) / eps;
    end
end