clear;
clc;
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

%% Initialize Kalman matrices
% Define model parameters
Ts = 0.02;
K = 1.3860;
eta = 11.0669;
omega = 7.8944;
b = 0.0735;

% State matrix A
Ac = [-eta, 0, 0;
      0, 0, 1;
      omega^2, -omega^2, -2*b*omega];

% Input matrix B
Bc = [K*eta; 0; 0];

% Output matrix C
Cc = [0, 1, 0];

sys = ss(Ac,Bc,Cc,0);
sysd = c2d(sys, Ts);

[A, B, C, ~] = ssdata(sysd);

n = size(A, 1);
r = size(B, 2);
m = size(C, 1);

% --- Augmented system for integral action ---
D = [1]; % Disturbance (the discrepancy between the model and real system)

d = size(D, 1);

A_tilde = [A, zeros(n, d);
           zeros(d, n), D];
B_tilde = [B; zeros(r, d)];

C_tilde = [C, eye(d)];

R=0.01; % measurement noise covariance
Q=diag([0.1;0.1;0.1;0.1]);  % process noise covariance

% Kalman initial
P=zeros(size(Q));
x_hat=zeros(size(Q,1),1);

%% Simulate the model
sim("sim_pendulum_v02.slx");

%% Load the variables from the logsout var
ts = logsout.get("t").Values.Data;
ys = rad2deg(logsout.get("phi").Values.Data);
dys = logsout.get("dphi").Values.Data;
ddys = logsout.get("ddphi").Values.Data;
us = logsout.get("u").Values.Data;

if isscalar(us)
    us = ones(size(ts))*us;
end

load("data/test_sim");
t = logsout.t;
tidx = find(t > 6.55, 1);
tmask = tidx:numel(t);
t = t(tmask);
y = logsout.y(tmask);
u = logsout.u(tmask);
t = t - t(1);
tend = find(t >= 30 - Ts, 1);
t = t(1:tend);
y = y(1:tend);
u = u(1:tend);

ysint = interp1(ts, ys, t);
%%
% ===========================
%   Plot Results
% ===========================
figure(1); clf;
hold on;
stairs(t, y);
stairs(ts, ys);
stairs(t, ysint);
title("Comparision: Real-Time vs Simulated System Response");
subtitle("MSE: " + num2str(mean((y - ysint).^2)))
xlabel("t [s]");
ylabel("$\varphi [^\circ]$", "Interpreter","latex");
legend("y","ysim", "yint", 'Location', 'southeast');
grid minor;
grid on;
hold off;

return;
%% Control input
figure(999); clf;
style='-k';
hold on
stairs(t, u,'LineWidth',1.5);
stairs(ts, us,'LineWidth',1.5);
legend("u","usim");
ylabel('u(k) [%]'); xlabel('t [s]'); grid on

% xlim([0,max(LOG_STEP)]);
hold off
set(gcf,'position',[200,400,650,400]);

