addpath("../misc")
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");
addpath("../misc/plotting")

%% Do the simulation

Ts = 0.05;
TIME_PER_STEP = 15;
U_STEPS = numel(0:100);

Tstop = TIME_PER_STEP * U_STEPS;
t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

pendulum = Pendulum();
% load("../misc/models/ipendulum_model");
load("apptest_model.mat");
pendulum = sys;
w_disturbance = false;

[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear_w_propeller(Ts, w_disturbance);

n = pendulum.n;
m = pendulum.m;

%% Initial state of the system
x0 = zeros(n, 1);

%% Initiate Extended Kalman Filter
[Q, R] = QR_matrix(n, m, true);

P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P,"epstol",Ts);

fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");

%% Create the control input signal
u = zeros(1, nsteps);
ustep = 0;

for step=1:nsteps
    u(step) = ustep;
    if step > 1 && mod(t(step), TIME_PER_STEP) == 0
        ustep = ustep + 1;
    end
end

%% Simulate the system and observer
in_disturbance = 1;
[x, y, xhat, yhat, sim_steps] = simulate_sys_ekf(t, u, f, h, ekf, x0, in_disturbance, Q, R, deg2rad([-60 210]));

t = t(1:sim_steps);
u = u(:, 1:sim_steps);

%% Plot
x1data = Data2Plot(t, rad2deg(x(1, :)), [], rad2deg(xhat(1, :)), "stairs", "s", "deg", "Pendulum state x1", "Plot of pendulum x1", false, "s", "all", [], true, [0 0 17 13.6]);
[fig, ax1, ~] = x1data.plotoutnerror(1, 0, "images/nss/state_x1_w_estimate_error");

% hold(ax1, "on");
% plot(ax1, t, LOG_REF, '--r', 'LineWidth', 1);
% hold(ax1, "off");

x2data = Data2Plot(t, rad2deg(x(2, :)), [], rad2deg(xhat(2, :)), "stairs", "s", "\frac{deg}{s}", "Pendulum state x2", "Plot of pendulum x2", false, "s", "all", [], true, [0 0 17 13.6]);
x2data.plotoutnerror(2, 0, "images/nss/state_x2_w_estimate_error");

if n == 3
    x3data = Data2Plot(t, rad2deg(x(3, :)), [], rad2deg(xhat(3, :)), "stairs", "s", "deg", "Pendulum angular position deviation estimate", "Plot of pendulum angle deviation", false, "s", "all", [], true, [0 0 17 13.6]);
    x3data.plotoutnerror(3, 0, "images/nss/state_x3_w_estimate_error");
end

ydata = Data2Plot(t, rad2deg(y), [], rad2deg(yhat), "stairs", "s", "deg", "Pendulum angular position", "Plot of pendulum angle", false, "s", "all", [], true, [0 0 17 13.6]);
[fig, ax1, ~] = ydata.plotoutnerror(4, 0, "images/nss/angular_position_w_estimate_error");

%% Plot simulated static char.
us = [];
ys = [];
last_u = -1;
i = 0;
yint = 0;
step_skip = 10;
cskip = 0;

for step=1:sim_steps
    if step + cskip > sim_steps
        break;
    end
    i = i + 1;
    uc = u(step + cskip);
    yc = y(step + cskip);
    yint = yint + yc;

    if uc > last_u
        us = [us, uc];
        ys = [ys, yint/i];
        cskip = cskip + step_skip;
        i = 0;
        yint = 0;
        last_u = uc;
    end
end

staticdata = Data2Plot(us, rad2deg(ys), [], [], "scatter", "\%PWM", "deg", "Simulated static characteristic", "Plot of the simulated static characteristic", false, "s", "all", [], false, [0 0 17 13.6]);
[fig, ax1] = staticdata.plotx(5, "u", "\varphi", "images/nss/simulated_static_char");
