clear;
clc;
%% Simulation parameters
L = 0.1; % m
R = 0.02; % m
m1 = 0.035; % kg
m2 = 0.095; % kg
g = 9.81; % m/s^2
b = 0.0004; % damping coefficient

% Helpful variables
L_2 = L/2;
LR = L + R;
L2_4 = L_2^2;
LR2 = LR^2;


c1 = m1*L_2 + m2 * LR;
c2 = m1*L2_4 + m2 * LR2;

x0 = deg2rad(180);
dx0 = 0;

%% Simulate the model
sim("sim_pendulum.slx");

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

