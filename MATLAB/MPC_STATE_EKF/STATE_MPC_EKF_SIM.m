clear; close all;
clc;

%%
addpath("../misc")
addpath("../misc/KF");
addpath("../misc/MPC");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");
addpath("../misc/plotting");

%% Do the simulation
Ts = 0.05;
Tstop = 30;
SYNC_TIME = 10; % [s]

U_PB = 30; % %PWM

nsteps_solo = floor(Tstop/Ts);
Tstop = Tstop + SYNC_TIME;

t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

pendulum = Pendulum();
load("../misc/models/ipendulum_model");
pendulum = sys;
% [A, B, C, D] = pendulum.ss_discrete(Ts);
[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, false);

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
x = zeros(size(x0,1), nsteps);
x(:, 1) = x0;
y = zeros(1, nsteps);
y(1) = x0(1);


%% MPC Setup

p = 15; % Prediction horizon

options = optimoptions("quadprog", ...
                       'ConstraintTolerance', 1e-5, ...
                       'OptimalityTolerance',1e-5, ...
                       'Display', 'off');

% Get the Jacobians of the nonlinear ss
if exist("Fx","var") && exist("Hx","var") && exist("Bu", "var")
    A = Fx(0, x0, 0);
    C = Hx(0, x0, 0);
    B = discrete_jacobian_u(f, 0, x0, 0, Ts);
end

% --- Augmented system for disturbance ---

D = [1]; % Disturbance (the discrepancy between the model and real system)

d = size(D, 1);

A_tilde = [A, zeros(n, d);
           zeros(d, n), D];
B_tilde = [B; zeros(r, d)];

C_tilde = [C, eye(d)];

n = 3;
x0 = [x0; 0];

% A_tilde = A;
% B_tilde = B;
% C_tilde = C;

% --- MPC weighting matrices ---
Q_mpc_=[15 0;
        0 0.75];
% Q_mpc_=[1];
R_mpc=[0.6];
Qd_mpc = [0.001];

Q_mpc = [Q_mpc_ zeros(size(Q_mpc_, 1), size(Qd_mpc, 2));
        zeros(size(Qd_mpc, 1), size(Q_mpc_, 2)), Qd_mpc];

% Q_mpc = Q_mpc_;

Q_ = diagblock(Q_mpc, p);
R_ = diagblock(R_mpc, p);

% MPC prediction matrices
% [M,N] = mpcfillmnoutput(A_tilde,B_tilde,C_tilde,p);  % build prediction matrices
[M, N] = mpcfillmnstate(A_tilde, B_tilde, p);
[Gamma] = mpcfillgamma(r,p);

% Quadratic cost Hessian
H = 2*(Gamma'*N'*Q_*N*Gamma + R_);
H = (H+H')/2; % for symmetry

% Control input constraints
u_lower = [-U_PB];
u_upper = [U_PB];

U_lower = repmat(u_lower, p, 1);
U_upper = repmat(u_upper, p, 1);

x_lower = deg2rad([-100; -300; -100]);
x_upper = deg2rad([100; 300; 100]);

X_lower = repmat(x_lower, p, 1);
X_upper = repmat(x_upper, p, 1);

y_lower = [-pi/3];
y_upper = [7*pi/6];

Y_lower = repmat(y_lower, p, 1);
Y_upper = repmat(y_upper, p, 1);

% Constraint matrix for quadratic programming
I = eye(size(Gamma));
A_con = [Gamma;
        -Gamma;
         N*Gamma;
        -N*Gamma];


%% LOOP
U = zeros(nsteps, 1);


REF_INIT = 30;
REF = REF_INIT; % deg
REF_STEPS = [-10, -5, 0, 10, -REF_INIT];
nsteps_per_ref = floor(nsteps_solo / (numel(REF_STEPS) + 1));

x_hat = x0;
y_hat = x0(1);
z = 0;
u = U_PB;

LOG_REF(1) = REF;

step_init = 0;

w_noise = 1;

% MPC predicted control inputs ones vector
u_ones = ones(p, r);

%% Kalman setup
[Q, R] = QR_matrix(n, m);

P = diag(ones(size(x0))*var(x0));

% ekf = ExtendedKalmanFilter(f,h,x0,1,'Q',Q,'R',R,'P0',P,"epstol", Ts);
ekf = KalmanFilter(A_tilde,B_tilde,C_tilde, "Q", Q, "R", R, "x0", x0, "P0", P);



fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");

ekf_yhat = zeros(1, nsteps);
ekf_x = zeros(n, nsteps);

ekf_yhat(1) = x0(1);
ekf_x(:, 1) = x0;

n = 2;
[Q, R] = QR_matrix(n, m);


%% Loop
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

        u_pred = u_ones*u;

        x_test = x_hat;
        x_test(1) = deg2rad(REF) - x_test(3);
        x_test(3) = 0;
        
        % Y_ref = repmat(deg2rad(REF), p, 1); % output
        Y_ref = repmat(x_test, p, 1);
        
        Mx = M*(x_hat);
        Nu = N*u_pred;
        b = 2*(Mx + Nu - Y_ref)'*Q_*N*Gamma;
        b_con = [U_upper - u_pred;
                -U_lower + u_pred;
                X_upper - Mx - Nu;
                -X_lower + Mx + Nu];
                % Y_upper - Mx - Nu;
                % -Y_lower + Mx + Nu]; % output

        % tic
        delta_U = quadprog(H, b, A_con, b_con, [], [], [], [], [], options);
        % toc

        if ~isempty(delta_U)
            ux = delta_U(1:r, :);
        end
        u = U_PB + ux;

    else
        u = U_PB;
    end
    
    
    w = chol(Q) * randn(n, 1) * w_noise;
    v = chol(R) * randn(m, 1) * w_noise;

    x(:, step) = f(t(step-1), x(:, step-1), u) + w;
    y(:, step) = h(t(step-1), x(:, step), u) + v;


    % [ekf, y_hat] = ekf.step(t(step-1), u, y(:, step)); % ekf
    [ekf, y_hat] = ekf.step(u, y(:, step)); % kf
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
ylabel(ax1, "u [%PWM]");
xlabel(ax1, "t [s]");
saveplot2file(fig,"images/LQR_SIM/control_input")
