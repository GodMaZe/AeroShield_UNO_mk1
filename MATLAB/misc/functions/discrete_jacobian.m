function J = discrete_jacobian(f, x, u, eps)
% Calculate the numerical finite-difference Jacobian of f(x)
    fx = f(x, u);
    m = numel(fx);
    n = numel(x);
    J = zeros(m, n);
    for i = 1:n
        x1 = x;
        x1(i) = x1(i) + eps; % (x + h)
        fx1 = f(x1, u); % f(x + h)
        % df/dx = \lim_{h -> 0} \frac{f(x + h) - f(x)}{h}
        J(:, i) = (fx1 - fx) / eps;
    end
end