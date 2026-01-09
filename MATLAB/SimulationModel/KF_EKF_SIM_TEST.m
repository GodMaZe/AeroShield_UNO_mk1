close all;
clear; clc;

addpath("../misc");
addpath("../misc/functions");
addpath("../misc/KF");
addpath("../misc/models");

%% Simulation parameters
x0 = deg2rad(45);
dx0 = 0;

mT = 12.03/1000; % kg

Ts = 0.05;

R = deg2rad(0.015)^2; % Measurement noise (from datasheet)
Q = diag([deg2rad(0.01)^2 deg2rad(Ts)^2]);
x0 = [x0;dx0];
P0 = eye(2)*var(x0);

%% Building the Filter objects
pendulum = Pendulum();

[A,B,C,D] = pendulum.ss_discrete(Ts);

Gsd = ss(A, B, C, D, Ts);

u = 0.05;

% f_cont = @(x, u) [x(2); -1/I_T * (xi*x(2) + mu*g*(m1 + m2)*tanh(x(2)) + G_1*sin(x(1)))] + [0; 1/I_T] * u(1);
% f = @(x,u) rk4_step(f_cont, x, u, Ts); % Important to use the rk4 step for precise and safe integration
% h = @(x,u) x(1);
% 
% Fx = @(x, u) discrete_jacobian(f_cont, x, u, Ts);
% Hx = @(x, u) [1, 0];

[f, h, Fx, Hx] = pendulum.nonlinear(Ts, false);

kf = KalmanFilter(A,B,C,'D',D,'Q',Q,'R',R,'x0',x0,'P0',P0);
ekf = ExtendedKalmanFilter(f,h,x0, size(B, 2), 'Q',Q,'R',R,'P0',P0,'epstol',Ts);
eekf = extendedKalmanFilter(f, h, x0, ...
    'StateCovariance', eye(2)*0.1, ...
    'ProcessNoise', Q, ...
    'MeasurementNoise', R, ...
    'StateTransitionJacobian', Fx, ...
    'MeasurementJacobian', Hx);

step_options = RespConfig;
step_options.Amplitude = u;
step_options.InitialState = x0;
step_options.Delay = 30;

% [y, t] = step(Gsd, step_options);
t = 0:Ts:60;
nsteps = numel(t);

xs = zeros(2, nsteps);
ys = zeros(nsteps, 1);

xs(:, 1) = x0;
u = 0;
for i=2:nsteps
    if Ts*i > 30
        u = 0.05;
    end
    selector = max(i - 1, 1);
    w = chol(Q) * randn(size(x0));
    xs(:, selector + 1) = f(t(i), xs(:, selector), u) + w;
    ys(selector) = h(t(i), xs(:, selector + 1), u);
end


ykf = zeros(nsteps, 1);
yekf = zeros(nsteps, 1);
yeekf = zeros(nsteps, 1);
xkf = zeros(nsteps, 2);
xekf = zeros(nsteps, 2);
xeekf = zeros(nsteps, 2);
uf = zeros(nsteps, 1);

%%
y = ys;
x = xs;
u = 0;
for i=2:nsteps
    if Ts*i > 30
        u = 0.05;
    end
    selector = max(i - 1, 1);
    [kf, yhat] = kf.step(u, y(selector));
    [ekf, yhate] = ekf.step(u, y(selector));

    % EKF step
    predict(eekf, u);
    correct(eekf, y(selector), u);

    yeekf(i) = eekf.State(1);
    ykf(i) = yhat;
    yekf(i) = yhate;

    xkf(i, :) = kf.xhat';
    xekf(i, :) = ekf.xhat';
    xeekf(i, :) = eekf.State';
    uf(i) = u;
end

%% Plot the results

MSE_EKF = (mean(rad2deg(y - yekf).^2));
MSE_KF = (mean(rad2deg(y - ykf).^2));
MSE_EEKF = (mean(rad2deg(y - yeekf).^2));

figure;
hold on;
stairs(t, rad2deg(y), 'k');
stairs(t, rad2deg(ykf), 'r--');
% plot(t, movmean(rad2deg(ykf), 10));
stairs(t, rad2deg(yekf), 'g-.');
stairs(t, rad2deg(yeekf), 'm--');
% plot(t, rad2deg(ys), 'b');
% stairs(t, yfx, 'k');
legend('Gs', "KF: " + num2str(MSE_KF), "EKF: " + num2str(MSE_EKF), "EEKF: " + num2str(MSE_EEKF), 'y');
xlabel('Time (s)');
ylabel('Output');
title('Kalman Filter vs Extended Kalman Filter');
grid minor;
grid on;
hold off;

figure;
hold on;
% Plot the state estimates
stairs(t, xkf(:, 2), 'r--', 'DisplayName', 'KF2');
stairs(t, xekf(:, 2), 'g-.', 'DisplayName', 'EKF2');
stairs(t, xeekf(:, 2), 'm--', 'DisplayName', 'EEKF2');
stairs(t, xs(2, :), 'b.', 'DisplayName', 'dy');
stairs(t, uf, '--k', 'DisplayName', 'u')
legend show;
grid minor;
grid on;
xlabel('t (s)');
ylabel('x2 (rad/s)');
title('State Estimates from Filters');
hold off;