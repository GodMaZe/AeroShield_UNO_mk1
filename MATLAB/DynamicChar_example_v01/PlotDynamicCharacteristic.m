clear;
close all; clc;

load("data\d1.mat");


t = dyn_char.t;
u = dyn_char.u;
y = dyn_char.y;


figure(111);
hold on;
plot(t, u);
plot(t, y);
legend('u','y');
grid minor;
hold off;

figure(222);
plot(t, y);