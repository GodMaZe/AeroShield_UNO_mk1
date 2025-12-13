close all;
clear; clc;

%% Simulation parameters
L = 0.1005; % m
R = 0.0175; % m
m1 = 0.0540; % kg
m2 = 0.0965; % kg
g = 9.81; % m/s^2
xi = 0.0003; % damping coefficient
mu = 0.00015;

% Define simulation v02 parameters
I_T = 1/3*m1*L^2 + 2/5*m2*(R)^2 + m2*(L+R)^2;
M_1 = m1*L/2 + m2*(L+R);
G_1 = g*M_1;

% Helpful variables
L_2 = L/2;
LR = L + R;
L2_4 = L_2^2;
LR2 = LR^2;


c1 = m1*L_2 + m2 * LR;
c2 = m1*L2_4 + m2 * LR2;

x0 = deg2rad(45);
dx0 = 0;

Ts = 0.05;

A = [0 1;
    -G_1/I_T -xi/I_T];
B = [0; 1/I_T];
C = [1 0];
D = 0;

Q = [0.1 0;
     0 0.1];
R = 0.01;
x0 = [x0;dx0];
P0 = eye(2)*var(x0);

%% Building the Filter objects
Gs = ss(A, B, C, D);
Gsd = c2d(Gs, Ts);

[A, B, C, D] = ssdata(Gsd);

f = @(x,u) [x(2); -1/I_T * (xi*x(2) + G_1*sin(x(1)))] + [0; 1/I_T] * u(1);
h = @(x,u) x(1);

kf = KalmanFilter(A,B,C,'D',D,'Q',Q,'R',R,'x0',x0,'P0',P0);
ekf = ExtendedKalmanFilter_gen(f,h,x0,'Q',Q,'R',R,'P0',P0);

u = 0.14;

step_options = RespConfig;
step_options.Amplitude = u;
step_options.InitialState = x0;

[y, t] = step(Gsd, step_options);

nsteps = numel(t);

ykf = zeros(nsteps, 1);
yekf = zeros(nsteps, 1);
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
    ykf(i) = yhat;
    yf = yhat;
    yekf(i) = yhate;
end

%% Plot the results
figure;
hold on;
stairs(t, rad2deg(y), 'b');
stairs(t, rad2deg(ykf), 'r--');
plot(t, movmean(rad2deg(ykf), 10));
% stairs(t, yekf, 'g-.');
% stairs(t, yfx, 'k');
legend('Gs', 'KF', 'EKF');
xlabel('Time (s)');
ylabel('Output');
title('Kalman Filter vs Extended Kalman Filter');
grid minor;
grid on;
hold off;
