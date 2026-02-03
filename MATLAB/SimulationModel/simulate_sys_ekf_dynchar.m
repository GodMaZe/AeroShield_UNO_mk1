function [t, u, x, y, xhat, yhat, simulation_steps] = simulate_sys_ekf(t, u, f, h, ekf, x0, w_disturbance, Q, R, YLIM, U_PB, U_STEP_SIZE, U_DT, SYNC_TIME)
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
    YLIM = [];
    U_PB = 30;
    U_STEP_SIZE = 5;
    U_DT = 1;
    SYNC_TIME = 10;
end

arguments (Output)
    t;
    u;
    x;
    y;
    xhat;
    yhat;
    simulation_steps;
end

is_u_empty = isempty(u);

u0 = 0;

if ~is_u_empty
    u0 = u(:, 1);
end

t = t + SYNC_TIME;
Ts = mean(diff(t));
t = [0:Ts:SYNC_TIME, t];

nsteps = numel(t);

n = size(x0, 1);

y0 = h(t(1), x0, u0);

m = size(y0, 1);

y = zeros(m, nsteps);
x = zeros(n, nsteps);
xhat = zeros(n, nsteps); % Not necessarely true, can estimate the disturbances as an extra state.
yhat = zeros(m, nsteps);

y(:, 1) = y0;
x(:, 1) = x0;
xhat(:, 1) = x0;
yhat(:, 1) = y0;
simulation_steps = 1;

w_limits = ~isempty(YLIM);

if is_u_empty
    u = [];
end

is_init = true;
uc = u0;

for step=1:nsteps-1
    nstep = step + 1;

    if is_u_empty
        if is_init
            uc = uc + U_DT;
            uc = max(0, min(uc, U_PB));
            if uc >= U_PB
                is_init = false;
            end
        else
            uc = U_PB;
        end
    
        elapsed = t(step) - SYNC_TIME;
    
        if elapsed >= 0
            uc = U_PB + U_STEP_SIZE;
        end

        u = [u, uc];
    else
        uc = u(:, step); % Use the control input from the input matrix
    end

    w = chol(Q) * randn(n, 1) * w_disturbance;
    v = chol(R) * randn(m, 1) * w_disturbance;

    x(:, nstep) = f(t(step), x(:, step), u(:, step)) + w;
    y(:, nstep) = h(t(nstep), x(:, nstep), u(:, step)) + v;
    cy = y(:, nstep);

    [ekf, y_hat] = ekf.step(t(step), u(:, step), y(:, nstep));
    xhat(:, nstep) = ekf.get_xhat();
    yhat(:, nstep) = y_hat;

    simulation_steps = simulation_steps + 1;

    if w_limits && (min(YLIM) > cy || max(YLIM) < cy)
        sim_range = 1:simulation_steps;
        y = y(:, sim_range);
        x = x(:, sim_range);
        yhat = yhat(:, sim_range);
        xhat = xhat(:, sim_range);
        t = t(sim_range);
        u = u(:, sim_range);
        break;
    end
end

end