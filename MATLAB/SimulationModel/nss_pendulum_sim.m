addpath("../misc")
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");

%% Do the simulation
Ts = 0.05;
Tstop = 20;
t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

u = 0.00;
pendulum = Pendulum();
[f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, false);
[A,B,C,D] = pendulum.ss_discrete(Ts);

x0 = [0; 0];
x1 = x0(1);
x2 = x0(2);
x = zeros(size(x0,1), nsteps);
x(:, 1) = x0;

R = deg2rad(0.015)^2; % Measurement noise (from datasheet)
Q = diag([deg2rad(0.01)^2 deg2rad(0.01^2/Ts)]);

% x0 = [0; 0];
P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P,"epstol",Ts);

fprintf("Init EKF:\n\n");
disp(ekf.xhat);
fprintf("========\n");

% ekf = KalmanFilter(A,B,C,'Q',Q,'R',R,'x0',x0,'P0',P);

ekf_yhat = zeros(nsteps, 1);
ekf_x = zeros(size(x));

ekf_yhat(1) = x0(1);
ekf_x(:, 1) = x0;

U = zeros(nsteps, 1);

for step=2:nsteps
    
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

    

    if step == 2
        u = 100;
    else
        u = 0;
    end

    U(step) = u;
    
    w = chol(Q) * randn(2, 1) * 0;
    x(:, step) = f(t(step-1), x(:, step-1), u) + w;

    [ekf, yhat] = ekf.step(t(step-1), u, x(1, step));
    
    ekf_yhat(step) = yhat;
    ekf_x(:, step) = ekf.xhat;

    if step < 5
        disp(ekf.xhat)
    end
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
ylabel(ax2, "$\Delta x_1^{2}\ \left[rad\right]^2$", "Interpreter", "latex");
grid minor;
grid on;



figure(2); clf;
tiledlayout(3,1,"TileSpacing","compact","Padding","tight");
ax1 = nexttile([2 1]);
hold on;
stairs(ax1, t, (x(2, :)));
plot(ax1, [0, t(end)], [0, 0], '--r');
stairs(ax1, t, (ekf_x(2, :)));
% stairs(ax1, t, U/max(U) * mean(abs(x(2,:))))
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