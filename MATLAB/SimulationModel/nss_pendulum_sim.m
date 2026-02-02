addpath("../misc")
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");
addpath("../misc/plotting")

%% Do the simulation

Ts = 0.05;
Tstop = 30;
t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

pendulum = Pendulum();
load("../misc/models/ipendulum_model");
% load("apptest_model.mat");
pendulum = sys;
w_disturbance = false;

[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, w_disturbance);
[A,B,C,D] = pendulum.ss_discrete(Ts);

n = 2 + w_disturbance;
m = pendulum.m;

%% Initial state of the system
x0 = zeros(n, 1);

%% Initiate Extended Kalman Filter
R = (0.015)^2; % Measurement noise (from datasheet)
if size(x0, 1) == 3
    Q = diag([(0.001)^2 (0.001*Ts)^2 0.1]);
else
    Q = diag([(0.001)^2 (0.001*Ts)^2]);
end

P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P,"epstol",Ts);

fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");

%% Create the control input signal
u = zeros(1, nsteps);
u(1) = 25;
u(2) = 80;
u(3) = -15;
u(4) = 5;

%% Simulate the system and observer
in_disturbance = 0;
[x, y, xhat, yhat] = simulate_sys_ekf(t, u, f, h, ekf, x0, in_disturbance, Q, R);

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

%% Comparison to the measured impulse characteristic
load("mean_impulse");

figure(777); clf;
hold on;
stairs(t, u, '-k', 'LineWidth', 1);
stairs(tt, ut, 'LineWidth', 1);
legend('Measured Impulse', 'Simulated Pendulum', 'Location', 'Best');
xlabel('t [s]');
ylabel("$\varphi [^\circ]$", 'Interpreter', 'latex');
title('Pendulum Angle Comparison');
grid minor;
grid on;
hold off;

figure(666); clf;
hold on;
plot(t, y, '-k', 'LineWidth', 1);
plot(tt, rad2deg(yt), 'LineWidth', 1);
legend('Measured Impulse', 'Simulated Pendulum', 'Location', 'Best');
xlabel('t [s]');
ylabel("$\varphi [^\circ]$", 'Interpreter', 'latex');
title('Pendulum Angle Comparison');
grid minor;
grid on;
hold off;

yhat_ = yt(1:numel(t));
tt_ = tt(1:numel(t));
ycdata = Data2Plot(t', y', tt_, rad2deg(yhat_), "plot", "s", "deg", "Pendulum angular position", "Impulse response comparision", false, "s", "all", [], true);
[fig, ax1, ~] = ycdata.plotoutnerror(5, 0, "images/nss/angular_position_impulse_compare");

