function [x, y] = simulate_sys(t, u, f, h, x0, w_disturbance, Q, R)
%SIM_SYSEKF Simulate a system defined in a nonlinear state space using the
%vector function f, with the measurement function h. (Optionally) Provided with the
%initial system state.
arguments (Input)
    t;
    u;
    f;
    h;
    x0;
    w_disturbance = false;
    Q = ones(1, 1);
    R = ones(1, 1);
end

arguments (Output)
    x;
    y;
end

nsteps = numel(t);

n = size(x0, 1);

y0 = h(t(1), x0, u(:, 1));

m = size(y0, 1);

y = zeros(m, nsteps);
x = zeros(n, nsteps);

y(:, 1) = y0;
x(:, 1) = x0;

for step=1:nsteps-1
    nstep = step + 1;
    w = chol(Q) * randn(n, 1) * w_disturbance;
    v = chol(R) * randn(m, 1) * w_disturbance;

    x(:, nstep) = f(t(step), x(:, step), u(:, step)) + w;
    y(:, nstep) = h(t(nstep), x(:, nstep), u(:, step)) + v;
end

end