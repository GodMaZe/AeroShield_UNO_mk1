close all;
clear; clc;

addpath("../misc");
addpath("../misc/functions");
addpath("../misc/KF");
addpath("../misc/models");

%% Simulation parameters
x0 = deg2rad(45);
dx0 = 0;

Ts = 0.05;

Q = [0.01 0;
     0 0.01];
R = 0.01;
x0 = [x0;dx0];
P0 = eye(2)*var(x0);

%% Building the Filter objects
pendulum = Pendulum(100);

[A,B,C,D] = pendulum.ss_discrete(Ts);

Gsd = ss(A, B, C, D, Ts);

u = 0.0;

% f_cont = @(x, u) [x(2); -1/I_T * (xi*x(2) + mu*g*(m1 + m2)*tanh(x(1)) + G_1*sin(x(1)))] + [0; 1/I_T] * u(1);
% f = @(x,u) rk4_step(f_cont, x, u, Ts); % Important to use the rk4 step for precise and safe integration
% h = @(x,u) x(1);
% 
% Fx = @(x, u) discrete_jacobian(f_cont, x, u, Ts);
% Hx = @(x, u) [1, 0];

[f, h, Fx, Hx] = pendulum.nonlinear(Ts);

kf = KalmanFilter(A,B,C,'D',D,'Q',Q,'R',R,'x0',x0,'P0',P0);
ekf = ExtendedKalmanFilter(f,h,x0, size(B, 2),'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P0,'epstol',Ts);
eekf = extendedKalmanFilter(f, h, x0, ...
    'StateCovariance', eye(2)*0.1, ...
    'ProcessNoise', Q, ...
    'MeasurementNoise', R, ...
    'StateTransitionJacobian', Fx, ...
    'MeasurementJacobian', Hx);

step_options = RespConfig;
step_options.Amplitude = u;
step_options.InitialState = x0;

[y, t] = step(Gsd, step_options);

nsteps = numel(t);

ykf = zeros(nsteps, 1);
yekf = zeros(nsteps, 1);
yeekf = zeros(nsteps, 1);
yfx = zeros(nsteps, 1);
xfx = zeros(nsteps, 2);
xf = x0;
yf = x0(1);

%%
for i=1:nsteps
    selector = max(i - 1, 1);
    % xf = xf + f(xf, u) * Ts;
    % yf = h(xf, u);
    % yfx(i) = yf;
    % xfx(i, :) = xf';
    [kf, yhat] = kf.step(u, y(selector));
    [ekf, yhate] = ekf.step(u, y(selector));
    % EKF step
    predict(eekf, u);
    correct(eekf, y(selector), u);

    yeekf(i) = eekf.State(1);
    ykf(i) = yhat;
    yf = yhat;
    yekf(i) = yhate;
end

%% Plot the results

MSE_EKF = (mean(rad2deg(y - yekf).^2));
MSE_KF = (mean(rad2deg(y - ykf).^2));
MSE_EEKF = (mean(rad2deg(y - yeekf).^2));

figure;
hold on;
stairs(t, rad2deg(y), 'b');
stairs(t, rad2deg(ykf), 'r--');
% plot(t, movmean(rad2deg(ykf), 10));
stairs(t, rad2deg(yekf), 'g-.');
stairs(t, rad2deg(yeekf), 'm--');
% stairs(t, yfx, 'k');
legend('Gs', "KF: " + num2str(MSE_KF), "EKF: " + num2str(MSE_EKF), "EEKF: " + num2str(MSE_EEKF));
xlabel('Time (s)');
ylabel('Output');
title('Kalman Filter vs Extended Kalman Filter');
grid minor;
grid on;
hold off;
