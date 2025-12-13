% Pendulum EKF example
rng(0);

% Parameters
g = 9.81;
L = 1.0;
dt = 0.01;
T = 10;
N = floor(T/dt);

% Continuous dynamics: x = [theta; omega]
f_cont = @(x) [x(2); - (g/L) * sin(x(1)) - 1*x(2)];

% Discrete transition via RK4
stateTransitionFcn = @(x) rk4_step(x, f_cont, dt);

% Measurement: measure angle only (nonlinear identity here)
measurementFcn = @(x) x(1);

% Analytical Jacobians
Fjac = @(x) discreteJacobian(x, f_cont, dt);
Hjac = @(x) [1 0];

% Initial true state and EKF initial state
x_true = [1.2; 0.0];         % true initial
x0_est = x_true + [0.1; 0.2];% initial estimate

% Noise covariances
Q = diag([1e-6, 1e-4]);      % process noise (state)
R = 0.01;                    % measurement noise (scalar)

% Create extendedKalmanFilter
ekf = extendedKalmanFilter(stateTransitionFcn, measurementFcn, x0_est, ...
    'StateCovariance', eye(2)*0.1, ...
    'ProcessNoise', Q, ...
    'MeasurementNoise', R, ...
    'StateTransitionJacobian', Fjac, ...
    'MeasurementJacobian', Hjac);

% Preallocate
x_est = zeros(2,N);
x_est(:,1) = ekf.State;
x_tr  = zeros(2,N);
x_tr(:,1) = x_true;
z = zeros(1,N);

% Simulation loop
for k = 2:N
    % propagate true state (with small process noise)
    w = mvnrnd([0;0], Q)'; 
    x_true = rk4_step(x_true, f_cont, dt) + w;
    x_tr(:,k) = x_true;

    % measurement (angle) with noise
    v = sqrt(R)*randn;
    z(k) = measurementFcn(x_true) + v;

    % EKF step
    predict(ekf);
    correct(ekf, z(k));

    x_est(:,k) = ekf.State;
end

% Compute RMSE
rmse = sqrt(mean((x_est - x_tr).^2,2));
fprintf('RMSE theta: %.4f rad, omega: %.4f rad/s\n', rmse(1), rmse(2));

%% Plot the results
t = 0:dt:T-dt;
figure;
hold on;
stairs(t, z, 'b');
stairs(t, x_est(1, :), 'r--');
% stairs(t, yekf, 'g-.');
% stairs(t, yfx, 'k');
legend('Sys', 'EKF');
xlabel('Time (s)');
ylabel('Output');
title('Kalman Filter vs Extended Kalman Filter');
grid minor;
grid on;
hold off;

% --- helper functions ---
function xnext = rk4_step(x, f, dt)
    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);
    xnext = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function Fd = discreteJacobian(x, f_cont, dt)
    % Numerical Jacobian of discrete map x_{k+1} = RK4(x)
    epsv = 1e-6;
    n = numel(x);
    Fd = zeros(n);
    fdisc = @(xx) rk4_step(xx, f_cont, dt);
    fx = fdisc(x);
    for i=1:n
        dx = zeros(n,1);
        dx(i) = epsv;
        Fd(:,i) = (fdisc(x+dx) - fx) / epsv;
    end
end
