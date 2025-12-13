% Periodic system test for Kalman filter
Ts = 0.05;               % sample period (s)
Tsim = 20;               % total simulation time (s)
N = round(Tsim / Ts);    % number of steps

% Base state-space (2 states, 1 input, 1 output)
A0 = [0.95 0.1; -0.05 0.9];
A1 = [0.02 -0.01; 0.01 -0.02]; % shape of time variation
B  = [0.1; 0.05];
C  = [1 0];
D  = 0;

% Periodic variation settings
Tperiod = 2.0;                       % period of parameter variation (s)
omega = 2*pi / Tperiod;
k = (0:N-1)';

% Process and measurement noise covariances
Q = 1e-4 * eye(2);   % process noise covariance
R = 5e-3;            % measurement noise covariance (scalar)

% Preallocate
x_true = zeros(2,N);
xhat   = zeros(2,N);
y      = zeros(1,N);
yhat   = zeros(1,N);
u      = zeros(1,1); % zero input; change if you want to test input tracking

% Initial conditions
x_true(:,1) = [0.1; -0.1];
% xhat   = [0; 0];
P = 1e-2 * eye(2);

kf = KalmanFilter(A, B, C, 'D', D, 'Q', Q, 'R', R, 'x0', xhat(:, 1), 'P0', P);

rng(0); % reproducible noise

for i = 1:N-1
    % time-varying A (periodic, smooth)
    t = (i-1)*Ts;
    alpha = 0.5*(1 + sin(omega * t));  % varies between 0 and 1
    A = A0 + alpha * A1;

    % simulate true system (discrete)
    w = mvnrnd([0;0], Q)';            % process noise
    x_true(:,i+1) = A * x_true(:,i) + B * u + w;

    % measurement
    v = sqrt(R) * randn;
    y(i) = C * x_true(:,i) + v + D*u;

    % ---- Kalman filter (time-varying) ----
    [kf, yhat_] = kf.step(u, y(i));
    xhat(:, i) = kf.xhat;  % Update the estimated state from the Kalman filter
    yhat(i) = yhat_;
end

% final measurement at N
t = (N-1)*Ts;
alpha = 0.5*(1 + sin(omega * t));
A = A0 + alpha * A1;
y(N) = C * x_true(:,N) + sqrt(R)*randn;

% Plot results
time = (0:N-1)' * Ts;
figure;
subplot(2,1,1)
plot(time, x_true(1,:),'-', time, xhat(1,:), '--');
ylabel('State 1'); legend('true','est'); grid on
subplot(2,1,2)
plot(time, x_true(2,:),'-', time, xhat(2,:), '--');
ylabel('State 2'); xlabel('Time (s)'); legend('true','est'); grid on

figure;
plot(time, y, '.-'); hold on
plot(time, C*xhat, '--');
ylabel('Output / Estimate'); xlabel('Time (s)'); legend('y (meas)','C*xhat'); grid on
