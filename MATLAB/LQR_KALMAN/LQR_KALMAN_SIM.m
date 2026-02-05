clear; close all;
clc;

%%
addpath("../misc")
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");
addpath("../misc/plotting");

%% Do the simulation
Ts = 0.02;
Tstop = 10;
SYNC_TIME = 0; % [s]

nsteps_solo = floor(Tstop/Ts);
Tstop = Tstop + SYNC_TIME;

t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

pendulum = Pendulum();
load("../misc/models/ipendulum_model");
pendulum = sys;
% [A, B, C, D] = pendulum.ss_discrete(Ts);
[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear_w_propeller(Ts, false);

if exist("pendulum", "var")
    n = pendulum.n;
    m = pendulum.m;
    r = pendulum.r;
else
    n = size(A, 1);
    r = size(B, 2);
    m = size(C, 1);   
end

x0 = zeros(n, 1);
x1 = x0(1);
x2 = x0(2);
x = zeros(size(x0,1), nsteps);
x(:, 1) = x0;
y = zeros(1, nsteps);
y(1) = x0(1);

[Q, R] = QR_matrix(n, m, true);

% x0 = [0; 0];
P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Q',Q,'R',R,'P0',P,"epstol", Ts);

fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");

ekf_yhat = zeros(nsteps, 1);
ekf_x = zeros(size(x));

ekf_yhat(1) = x0(1);
ekf_x(:, 1) = x0;

%% LQR Setup
% --- Augmented system for integral action ---
if exist("Fx","var") && exist("Hx","var") && exist("Bu", "var")
    xinit = zeros(n, 1);
    A = Fx(0, xinit, 0);
    C = Hx(0, xinit, 0);
    B = discrete_jacobian_u(f, 0, xinit, 0, Ts);
end

% --- Augmented system for integral action ---
B_tilde=zeros(n+m, r);

A_tilde = [A, zeros(n, m);
           -C, eye(m, m)];

B_tilde(1:n) = B;


Q_=diag([0.00001 10 0.00000001]);
R_=[0.01];
Qz=[15];

Q_tilde=[Q_, zeros(size(Q_, 1), size(Qz, 2));
        zeros(size(Qz, 1), size(Q_, 2)), Qz];

[P_LQ, ~, K_LQ] = dare(A_tilde, B_tilde, Q_tilde, R_);
K_LQ = -K_LQ;
Kx = K_LQ(1:n);
Kz = K_LQ((n+1):end);

%% LOOP
U = zeros(nsteps, 1);

U_PB = 30; % %PWM

REF_INIT = 30;
REF = REF_INIT; % deg
REF_STEPS = [+10, -REF_INIT];
nsteps_per_ref = floor(nsteps_solo / (numel(REF_STEPS) + 1));

x_hat = x0;
y_hat = x0(1);
z = 0;
u = U_PB;

LOG_REF(1) = REF;

step_init = 0;

w_noise = 1;

for step=2:nsteps

    LOG_REF = [LOG_REF, REF];

    if t(step) >= SYNC_TIME
        if step_init == 0
            step_init = step;
            istep = 1;
        else
            istep = istep + 1;
        end

        if mod(istep, nsteps_per_ref) == 0 && istep/nsteps_per_ref <= numel(REF_STEPS)
            REF = REF_INIT + REF_STEPS(istep/nsteps_per_ref);
        end

        if exist("ekf", "var")
            x_test = x_hat;
            A_new = discrete_jacobian(f, t(step-1), x_test, u, 1e-6);
            C_new = Hx(t(step-1), x_test, u);
            B_new = discrete_jacobian_u(f, t(step-1), x_test, u, 1e-6);

            % A = (A+A_new)/2;
            % B = (B+B_new)/2;
            % C = (C+C_new)/2;
            
            A = (2*A+1*A_new)/3;
            B = (2*B+1*B_new)/3;
            C = (2*C+1*C_new)/3;

            % A = A_new;
            % B = B_new;
            % C = C_new;

            A_tilde(1:size(A, 1), 1:size(A, 2)) = A;

            B_tilde(1:n, :) = B;

            % --- Solve Discrete-time Algebraic Riccati Equation ---
            [P_LQ,~,K_LQ] = dare(A_tilde, B_tilde, Q_tilde, R_);
            K_LQ = -K_LQ;

            Kx=K_LQ(1:n);           % state feedback part
            Kz=K_LQ(n + 1:end);        % integral feedback part
        end

        ux = Kx*(x_hat) + Kz*z;
        e = deg2rad(REF) - y(:, step - 1); % OPT Params
        z = z + e;

        % u = U_PB + saturate(ux, -U_PB, 100 - U_PB);
        u = saturate(ux, 0, 100);
    else
        u = U_PB;
    end
    
    
    w = chol(Q) * randn(n, 1) * w_noise;
    v = chol(R) * randn(m, 1) * w_noise;

    x(:, step) = f(t(step-1), x(:, step-1), u*0.8) + w;
    y(:, step) = h(t(step-1), x(:, step), u) + v;


    [ekf, y_hat] = ekf.step(t(step-1), u, y(:, step));
    x_hat = ekf.get_xhat();

    ekf_yhat(step) = y_hat;
    ekf_x(:, step) = ekf.xhat;

    U(step) = u;

    % if step < 5
    %     disp(ekf.xhat)
    % end
end

%% Plot

fprintf("Velocity mean: %f\n", mean(x(2, :)));

%% Plot the control input
figure(3); clf;
stairs(t, U);
xlabel('Time (s)');
ylabel('Control Input (u)');
title('Control Input Over Time');
grid on;

%% Testing new plotting class
x1data = Data2Plot(t, rad2deg(x(1, :)), [], rad2deg(ekf_x(1, :)), "stairs", "s", "deg", "Pendulum angular position", "Plot of pendulum angle", false, "s", "all", [], true, [0 0 17 13.6]);
[fig, ax1, ~] = x1data.plotoutnerror(1, 0, "angular_position_w_estimate_error");

hold(ax1, "on");
plot(ax1, t, LOG_REF, '--r', 'LineWidth', 1);
hold(ax1, "off");

x2data = Data2Plot(t, rad2deg(x(2, :)), [], rad2deg(ekf_x(2, :)), "stairs", "s", "\frac{deg}{s}", "Pendulum angular velocity", "Plot of pendulum velocity", false, "s", "all", [], true, [0 0 17 13.6]);
x2data.plotoutnerror(2, 0, "angular_velocity_w_estimate_error");

if n == 3
    x3data = Data2Plot(t, rad2deg(x(3, :)), [], rad2deg(ekf_x(3, :)), "stairs", "s", "^\circ", "Pendulum angular position deviation estimate", "Plot of pendulum angle deviation", false, "s", "all", [], true, [0 0 17 13.6]);
    x3data.plotoutnerror(3, 0, "angualr_position_deviation_w_estimate_error");
end

ycdata = Data2Plot(t, rad2deg([y; ekf_yhat']), [], [], "plot", "s", "deg", "Control response", "Control response", false, "s", "all", "simulation", true, [0 0 17 5.6]);
[fig, ax1] = ycdata.plotx(5, [], [], "images/LQR_SIM/angular_position");
hold(ax1, "on");
plot(ax1, t, LOG_REF, '--r', 'LineWidth', 1,"DisplayName", "ref");
xline(SYNC_TIME,"--k","DisplayName","Control start");
ylabel(ax1, "$\varphi\ [deg]$", "Interpreter", "latex");
xlabel(ax1, "$t\ [s]$", "Interpreter", "latex");
hold(ax1, "off");
saveplot2file(fig,"images/LQR_SIM/angular_position");

%%
ucdata = Data2Plot(t, U', [], [], "stairs", "s", "\%PWM", "Control input", "Control input", false, "s", "all", "simulation", true, [0 0 17 5.6]);
[fig, ax1] = ucdata.plotx(6, [], [], "images/LQR_SIM/control_input");
legend(ax1, "u", "Location", "southwest");
ylabel(ax1, "$u [\%PWM]$", "Interpreter", "latex");
xlabel(ax1, "t [s]");
saveplot2file(fig,"images/LQR_SIM/control_input")
