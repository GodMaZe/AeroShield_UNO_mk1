close all;
clear; clc;

load("data/ref1354.mat");


t = logsout.t;
y = logsout.y;
u = logsout.u;
r = logsout.r;
e = r - y;
dt = logsout.dt;


figure(111);
plot(t, y, '-k', 'LineWidth', 1.5);
hold on;
plot(t, r, '-r', 'LineWidth', 1.5);
title('Control Response');
subtitle("P = " + num2str(P) + ", I = " + num2str(I) + ", D = " + num2str(D));
legend('y(t)', 'ref(t)', "Location", "best");
xlabel('t [s]');
ylabel('y [deg]');
grid on;
hold off;

figure(222);
plot(t, u, '-k', 'LineWidth', 1);
title('Control Output');
subtitle("P = " + num2str(P) + ", I = " + num2str(I) + ", D = " + num2str(D));
legend('u(t)', "Location", "best");
xlabel('t [s]');
ylabel('u [%]');
grid on;

figure(666);
plot(t, y, '-k', 'LineWidth', 1.5);
hold on;
plot(t, r, '-r', 'LineWidth', 1.5);
plot(t, e, '-b', 'LineWidth', 1.5);
title('Control Error Response');
subtitle("P = " + num2str(P) + ", I = " + num2str(I) + ", D = " + num2str(D));
legend('y(t)', 'ref(t)', 'e(t)', "Location", "best");
xlabel('t [s]');
ylabel('Value [deg]');
grid minor;
hold off;


if exist("RegParams", "var")
    figure(777);
    plot(t, RegParams(:, 1), '-k', 'LineWidth', 1);
    hold on;
    plot(t, RegParams(:, 2), '-r', 'LineWidth', 1);
    plot(t, RegParams(:, 3), '-b', 'LineWidth', 1);
    title('Gain Scheduling');
    subtitle("PID Params");
    legend('P', 'I', 'D', "Location", "best");
    xlabel('t [s]');
    ylabel('Value [-]');
    grid minor;
    hold off;
end
