Ts = 0.05;
Tstop = 60;
t = 0:Ts:Tstop; % Create a time vector from 0 to Tstop with step Ts
nsteps = numel(t);

u = 0.00;
pendulum = Pendulum();
[f, h, Fx, Hx] = pendulum.nonlinear(Ts, false);

x0 = [pi/4; 0];
x1 = x0(1);
x2 = x0(2);
x = zeros(size(x0,1), nsteps);
x(:, 1) = x0;

for step=2:nsteps
    if step >= nsteps/2
        u = 0.05;
    end
    % X = x(:, step - 1);
    % x2 = X(2) + [0 1] * Ts * f(X, u);
    % X(2) = x2;
    % x1 = X(1) + [1 0] * Ts * f(X, u); % Update the first state variable
    % X(1) = x1;
    % x(:, step) = X;
    % 
    % x(:, step) = [x1; x2]; % Update the state vector
    % x(:, step) = rk4_step(f, x(:, step - 1), u, Ts);
    % x(:, step) = euler_step(f, x(:, step - 1), u, Ts);
    x(:, step) = f(x(:, step - 1), u);
end

figure(1); clf;
hold on;
plot(t, rad2deg(x));
plot([0, t(end)], [0, 0], 'k');
hold off;
xlabel('Time (s)');
ylabel('State Variables');
title('Pendulum State Over Time');
legend show;
grid minor;
grid on;

fprintf("Velocity mean: %f\n", mean(x(2, :)));
return;

% Parameters
m = 1;       % mass (kg)
k = 4;       % stiffness (N/m)
omega = sqrt(k/m);

% ODE function (anonymous)
springODE = @(t,y) [ y(2); -(k/m)*y(1) ];

% Initial conditions: x(0)=1 m, x'(0)=0 m/s
y0 = [1; 0];

% Time span
tspan = [0 10];

% Solve with ode45
[t,y] = ode45(springODE, tspan, y0);

% Plot displacement and velocity
figure(2); clf;
subplot(2,1,1)
plot(t,y(:,1))
ylabel('x (m)')
title('Mass-Spring Response')

subplot(2,1,2)
plot(t,y(:,2))
ylabel('v (m/s)')
xlabel('time (s)')
