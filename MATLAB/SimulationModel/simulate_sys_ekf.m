function [x, y, xhat, yhat] = simulate_sys_ekf(t, u, f, h, ekf, x0, w_disturbance, Q, R)
%SIM_SYSEKF Simulate a system defined in a nonlinear state space using the
%vector function f, with the measurement function h, while trying to
%estimate the state of the system using an observer. (Optionally) Provided with the
%initial system state.
arguments (Input)
    t;
    u;
    f;
    h;
    ekf;
    x0;
    w_disturbance = false;
    Q = ones(1, 1);
    R = ones(1, 1);
end

arguments (Output)
    x;
    y;
    xhat;
    yhat;
end

nsteps = numel(t);

n = size(x0, 1);

y0 = h(t(1), x0, u(:, 1));

m = size(y0, 1);

y = zeros(m, nsteps);
x = zeros(n, nsteps);
xhat = zeros(n, nsteps); % Not necessarely true, can estimate the disturbances as an extra state.
yhat = zeros(m, nsteps);

y(:, 1) = y0;
x(:, 1) = x0;
xhat(:, 1) = x0;
yhat(:, 1) = y0;

for step=1:nsteps-1
    nstep = step + 1;
    w = chol(Q) * randn(n, 1) * w_disturbance;
    v = chol(R) * randn(m, 1) * w_disturbance;

    x(:, nstep) = f(t(step), x(:, step), u(:, step)) + w;
    y(:, nstep) = h(t(nstep), x(:, nstep), u(:, step)) + v;

    [ekf, y_hat] = ekf.step(t(step), u(:, step), y(:, nstep));
    xhat(:, nstep) = ekf.get_xhat();
    yhat(:, nstep) = y_hat;
end

end