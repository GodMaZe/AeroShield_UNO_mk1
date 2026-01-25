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

u = 0.00;
pendulum = Pendulum();
load("../misc/models/ipendulum_model");
% load("apptest_model.mat");
pendulum = sys;
w_disturbance = false;

[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, w_disturbance);
[A,B,C,D] = pendulum.ss_discrete(Ts);

n = 2 + w_disturbance;

x0 = zeros(n, 1);
x1 = x0(1);
x2 = x0(2);
x = zeros(size(x0,1), nsteps);
x(:, 1) = x0;

R = (0.015)^2; % Measurement noise (from datasheet)
if size(x0, 1) == 3
    Q = diag([(0.01)^2 (0.01/Ts)^2 0.1]);
else
    Q = diag([(0.001)^2 (0.001*Ts)^2]);
end

% x0 = [0; 0];
P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P,"epstol",Ts);

fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");ss

% ekf = KalmanFilter(A,B,C,'Q',Q,'R',R,'x0',x0,'P0',P);

ekf_yhat = zeros(nsteps, 1);
ekf_x = zeros(size(x));

ekf_yhat(1) = x0(1);
ekf_x(:, 1) = x0;

U = zeros(nsteps, 1);

tx = 0;
cx = x0;

for step=1:nsteps
    
    % X = x(:, step - 1);
    % x2 = X(2) + [0 1] * Ts * f(X, u);
    % X(2) = x2;
    % x1 = X(1) + [1 0] * Ts * f(X, u); % Update the first state variable
    % X(1) = x1;
    % x(:, step) = X;
    % 
    % x(:, step) = [x1; x2]; % Update the state vector
    % x(:, step + 1) = rk4_step(f, t(step), x(:, step), u, Ts);
    % x(:, step) = euler_step(f, x(:, step - 1), u, Ts);

    if step == 1
        u = 100;
    else
        u = 0;
    end

    U(step) = u;
    
    w = chol(Q) * randn(size(Q,1), 1) * 1;
    x(:, step) = f(tx, cx, u) + w;

    [ekf, yhat] = ekf.step(tx, u, x(1, step));
    
    ekf_yhat(step) = yhat;
    ekf_x(:, step) = ekf.xhat;

    tx = tx + Ts;
    cx = x(:, step);

    if step < 5
        disp(ekf.xhat)
    end
end

%% Plot
x1data = Data2Plot(t, rad2deg(x(1, :)), rad2deg(ekf_x(1, :)), "stairs", "s", "deg", "Pendulum angular position", "Plot of pendulum angle", false, "s", "all", [], true, [0 0 17 13.6]);
[fig, ax1, ~] = x1data.plotoutnerror(1, 0, "images/nss/angular_position_w_estimate_error");

% hold(ax1, "on");
% plot(ax1, t, LOG_REF, '--r', 'LineWidth', 1);
% hold(ax1, "off");

x2data = Data2Plot(t, rad2deg(x(2, :)), rad2deg(ekf_x(2, :)), "stairs", "s", "\frac{deg}{s}", "Pendulum angular velocity", "Plot of pendulum velocity", false, "s", "all", [], true, [0 0 17 13.6]);
x2data.plotoutnerror(2, 0, "images/nss/angular_velocity_w_estimate_error");

if n == 3
    x3data = Data2Plot(t, rad2deg(x(3, :)), rad2deg(ekf_x(3, :)), "stairs", "s", "deg", "Pendulum angular position deviation estimate", "Plot of pendulum angle deviation", false, "s", "all", [], true, [0 0 17 13.6]);
    x3data.plotoutnerror(3, 0, "images/nss/angualr_position_deviation_w_estimate_error");
end

tt = t;

%% Comparison to the measured impulse characteristic
load("mean_impulse");

figure(777); clf;
hold on;
stairs(t, u, '-k', 'LineWidth', 1);
stairs(tt, U, 'LineWidth', 1);
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
plot(tt, rad2deg(x(1, :)), 'LineWidth', 1);
legend('Measured Impulse', 'Simulated Pendulum', 'Location', 'Best');
xlabel('t [s]');
ylabel("$\varphi [^\circ]$", 'Interpreter', 'latex');
title('Pendulum Angle Comparison');
grid minor;
grid on;
hold off;

