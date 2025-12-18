clear; close all; clc;

load("best_model_2o");

s = tf('s');
omega_0 = 1;
xi = 0.1;
Gs = omega_0^3 / (s^3 + 4*xi*omega_0*s^2 + 4*xi^2*omega_0^2*s + omega_0^3);

K = 1;
T1 = 0.5;
Gs = K/(T1*s+1);

a = P.Denominator{1};
b = P.Numerator{1};
TPs = -1./roots(a);
TP1 = TPs(1);
TP2 = TPs(2);
K = sum(b(end)/a(end));

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

%% Controller
Tw = 0.15;
Gyw = 1/(Tw*s + 1);


R = 1/Gs * Gyw/(1 - Gyw);

Kp = (TP1 + TP2)/K/Tw;
Ki = 1/K/Tw;
Kd = TP1*TP2/K/Tw;

G = R*Gs;

Gywd = c2d(Gyw, Ts);

Rd = 1/Gsd * Gywd/(1 - Gywd);

% Simulate the closed-loop response with the PID controller
sysClosedLoop = feedback(R * Gs, 1);
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
