addpath("../misc")
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");
addpath("../misc/plotting")

%% Do the simulation

Ts = 0.05;
Tstop = 20;
t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

pendulum = Pendulum();
% load("../misc/models/ipendulum_model");
load("apptest_model.mat");
pendulum = sys;
w_disturbance = false;

[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear_w_propeller(Ts, w_disturbance);
% [A,B,C,D] = pendulum.ss_discrete(Ts);

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
U_PB = 30;
U_STEP_SIZE = 5;
udt = 1;
SYNC_TIME = 10;

%% Simulate the system and observer
in_disturbance = 1;
[t, u, x, y, xhat, yhat, simulation_steps] = simulate_sys_ekf_dynchar(t, [], f, h, ekf, x0, in_disturbance, Q, R, deg2rad([-60 210]), U_PB, U_STEP_SIZE, udt, SYNC_TIME);

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


tt = t;
yt = y;
ut = u;

%% Comparison to the measured dynamic characteristic
load("data/dynamic_char");

t = logsout.tp;
y = logsout.y;
u = logsout.u;

uidx = find(u>U_PB, 1);
t = t(uidx:end);
y = y(uidx:end);
u = u(uidx:end);

utidx = find(ut>U_PB, 1);
tt_ = tt(utidx:end);
yt_ = yt(:, utidx:end);
ut_ = ut(:, utidx:end);

szdiff = numel(tt_) - numel(t);
if szdiff > 0
    sim_mask = 1:numel(tt_)-szdiff;
    tt_ = tt_(sim_mask);
    yt_ = yt_(:, sim_mask);
    ut_ = ut_(:, sim_mask);
end

dcdata = Data2Plot(t, y, tt_, rad2deg(yt_), "plot", "s", "deg", "Pendulum angular position comparison", "Step response comparison", false, "s", "all", "Measurement vs Simulation", true);
dcdata.plotoutnerror(5, 0, "images/nss/angular_position_dynamic_compare", true);
