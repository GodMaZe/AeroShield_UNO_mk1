clear; close all; clc;

s = tf('s');
omega_0 = 5;
xi = 0.1;
Gs = omega_0^2 / (s^2 + 2*xi*omega_0*s + omega_0^2);

% Define the time vector for simulation
Ts = 0.05;
t = 0:Ts:30;

% Discretize the continuous plant
Gsd = c2d(Gs, Ts);

% Simulate the step response of the system
[y, t] = step(Gsd, t);

% Plot the step response
figure(1); clf;
stairs(t, y);
xlabel('Time (s)');
ylabel('Response');
title('Step Response of the System');
grid on;

% Display the final value of the step response
finalValue = y(end);
% Calculate the settling time and overshoot
settlingTime = stepinfo(Gsd).SettlingTime;
overshoot = stepinfo(Gsd).Overshoot;
% Calculate the rise time
riseTime = stepinfo(Gsd).RiseTime;
% Calculate the bandwidth of the system
bandwidth = bandwidth(Gsd);
% Display the transfer function of the system
disp('Transfer Function G(s):');
disp(Gsd);
% Display the system's poles and zeros
poles = pole(Gsd);
zeros = zero(Gsd);
% Display the system's step response characteristics
disp('Step Response Characteristics:');
disp(['Final Value: ', num2str(finalValue)]);
disp(['Settling Time: ', num2str(settlingTime)]);
disp(['Overshoot: ', num2str(overshoot), '%']);
disp(['Rise Time: ', num2str(riseTime)]);
disp(['Bandwidth: ', num2str(bandwidth), ' rad/s']);
disp('Poles of the system:');
disp(poles);
disp('Zeros of the system:');
disp(zeros);

save("system", "Gs", "Gsd");

Tw = 0.3;
Gyw = 1/(Tw*s + 1);

Gywd = c2d(Gyw, Ts);

R = 1/Gsd * Gywd/(1 - Gywd);

% Simulate the closed-loop response with the PID controller
sysClosedLoop = feedback(R * Gsd, 1);
[yClosedLoop, tClosedLoop] = step(sysClosedLoop, t);

% Plot the closed-loop step response
figure(2); clf;
stairs(tClosedLoop, yClosedLoop);
xlabel('Time (s)');
ylabel('Closed-Loop Response');
title('Closed-Loop Step Response of the System');
grid on;

figure(3); clf;
hold on;
stairs(t, y, 'DisplayName', 'open');
stairs(tClosedLoop, yClosedLoop, 'DisplayName', 'closed');
xlabel('Time (s)');
ylabel('Closed-Loop Response');
title('Closed-Loop Step Response of the System');
grid minor;
grid on;
legend show;
