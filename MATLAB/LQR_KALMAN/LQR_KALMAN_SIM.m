clear; close all;
clc;

%%
addpath("../misc")
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");

%% Do the simulation
Ts = 0.1;
Tstop = 60;
t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

u = 0;
pendulum = Pendulum();
[f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, false);
[A, B, C, D] = pendulum.ss_discrete(Ts);

x0 = [-pi/3; 0];
x1 = x0(1);
x2 = x0(2);
x = zeros(size(x0,1), nsteps);
x(:, 1) = x0;

R = deg2rad(0.015)^2; % Measurement noise (from datasheet)
Q = diag([deg2rad(0.01)^2 deg2rad(0.01^2*Ts)]);

% x0 = [0; 0];
P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P,"epstol",Ts);

fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");

ekf_yhat = zeros(nsteps, 1);
ekf_x = zeros(size(x));

ekf_yhat(1) = x0(1);
ekf_x(:, 1) = x0;

%% LQR Setup
n = pendulum.n;
m = pendulum.m;
r = pendulum.r;

% --- Augmented system for integral action ---
B_tilde=zeros(n+m, r);

A_tilde = [A, zeros(n, m);
           -C, eye(m, m)];

B_tilde(1:n) = B;

Q_=diag([1 5]);
R_=[0.1];
Qz=[1000];

Q_tilde=[Q_, zeros(size(Q_, 1), size(Qz, 2));
        zeros(size(Qz, 1), size(Q_, 2)), Qz];

[P_LQ, ~, K_LQ] = dare(A_tilde, B_tilde, Q_tilde, R);
K_LQ = -K_LQ;
Kx = K_LQ(1:n);
Kz = K_LQ((n+1):end);

%% LOOP
U = zeros(nsteps, 1);

U_PB = 20; % %PWM

REF = 25; % deg

x_hat = x0;
y_hat = x0(1);
z = 0;

for step=2:nsteps

    if step >= nsteps/2
        REF = 35;
    end

    [ekf, y_hat] = ekf.step(t(step - 1), u, x(1, step - 1));
    y_hat = y_hat;
    x_hat = ekf.get_xhat();

    ekf_yhat(step) = y_hat;
    ekf_x(:, step) = ekf.xhat;

    if step > 0
        if exist("EKF", "var")
            A = Fx(t(step-1), x_hat, u);
            C = Hx(t(step-1), x_hat, u);
            B = discrete_jacobian_u(f, t(step-1), x_hat, u, Ts);

            A_tilde(1:size(A, 1), 1:size(A, 2)) = A;

            B_tilde(1:n, :) = B; 

            % --- Solve Discrete-time Algebraic Riccati Equation ---
            [P_LQ,~,K_LQ] = dare(A_tilde, B_tilde, Q_tilde, R_);
            K_LQ = -K_LQ;
            
            Kx=K_LQ(1:n);           % state feedback part
            Kz=K_LQ(n + 1:end);        % integral feedback part
        end

        ux = Kx*x_hat + Kz*z;
        e = deg2rad(REF) - y_hat; % EKF
        % e = REF - y_hat; % OPT Params
        z = z + e;

        u = U_PB + saturate(ux, -U_PB, 100-U_PB);
        % u = saturate(u, 0, 100);
    else
        u = U_PB;
    end
    
    w = chol(Q) * randn(2, 1) * 1;
    x(:, step) = f(t(step-1), x(:, step-1), u*0.9) + w;

    U(step) = u;

    % if step < 5
    %     disp(ekf.xhat)
    % end
end

%% Plot
SKIP_STEPS = 1;
select_mask = SKIP_STEPS:nsteps;
e_phi = (x(1, select_mask) - ekf_x(1, select_mask)).^2;
e_dphi = (x(2, select_mask) - ekf_x(2, select_mask)).^2;
RMSE_X1 = rad2deg(sqrt(mean(e_phi)));
RMSE_X2 = rad2deg(sqrt(mean(e_dphi)));


figure(1); clf;
tiledlayout(3,1,"TileSpacing","compact","Padding","tight");
ax1 = nexttile([2 1]);
hold on;
stairs(ax1, t, rad2deg(x(1, :)));
stairs(ax1, t, rad2deg(ekf_x(1, :)));
hold off;
ylabel(ax1, "$x_1\ \left[deg\right]$", "Interpreter", "latex");
title(ax1, 'Pendulum angular position');
subtitle(ax1, "RMSE: " + num2str(RMSE_X1) + " rad")
legend(ax1, "y","y_{ekf}");
grid minor;
grid on;

ax2 = nexttile;
errorbar(ax2, t(select_mask), zeros(size(e_phi)), e_phi, '.');
title(ax2, "Simulated and observed state x_1: difference squared");
xlabel(ax2, "t [s]");
ylabel(ax2, "$\Delta x_1^{2}\ \left[deg\right]^2$", "Interpreter", "latex");
grid minor;
grid on;



figure(2); clf;
tiledlayout(3,1,"TileSpacing","compact","Padding","tight");
ax1 = nexttile([2 1]);
hold on;
stairs(ax1, t, (x(2, :)));
plot(ax1, [0, t(end)], [0, 0], '--r');
stairs(ax1, t, (ekf_x(2, :)));
stairs(ax1, t, U/max(U) * mean(abs(x(2,:))))
hold off;
ylabel(ax1, "$x_2\ \left[\frac{rad}{s}\right]$", "Interpreter", "latex");
title(ax1, 'Pendulum angular velocity');
subtitle(ax1, "RMSE: " + num2str(RMSE_X2) + " rad s^{-1}")
legend(ax1, "dy","0-line","dy_{ekf}");
grid minor;
grid on;

ax2 = nexttile;
errorbar(ax2, t(select_mask), zeros(size(e_dphi)), e_dphi, '.');
title(ax2, "Simulated and observed state x_2: difference squared");
xlabel(ax2, "t [s]");
ylabel(ax2, "$\Delta x_2^{2}\ \left[\frac{rad}{s}\right]^2$", "Interpreter", "latex");
grid minor;
grid on;

fprintf("Velocity mean: %f\n", mean(x(2, :)));

%% Plot the control input
figure(3); clf;
stairs(t, U);
xlabel('Time (s)');
ylabel('Control Input (u)');
title('Control Input Over Time');
grid on;