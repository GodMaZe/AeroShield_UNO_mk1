close all;
clear; clc;

%%
FILENAME_DATA = "data/train";
FILENAME = "train_ident_model";
FILENAME_TRAIN = "train_ident";
FILENAME_TEST = "test_sys";

load(FILENAME);
load(FILENAME_DATA);

if ~exist(FILENAME_TRAIN, "dir")
    mkdir(FILENAME_TRAIN);
end

if ~exist(FILENAME_TEST, "dir")
    mkdir(FILENAME_TEST);
end

dt = mean(diff(logsout.tp));

%%
% Set the system
Gs = Gs21; %GS21

npoles = numel(Gs.den) - 1;
nzeros = numel(Gs.num);

Gsd = c2d(Gs, dt);

ndpoles = numel(Gsd.den) - 1;
ndzeros = numel(Gsd.num);

if Gsd.num(1) <= 0.001
    ndzeros = ndzeros - 1;
end

% Gs = Gsd;
% npoles = ndpoles;
% nzeros = ndzeros;

%%
t = 0:dt:5;
nsteps = numel(t);

u = zeros(nzeros, nsteps);
u(1, :) = 5; %*sin(5*t);

for i=2:nzeros
    u(i, :) = gradient(u(i-1,:), dt);
end

x = zeros(npoles, nsteps+1);
y = zeros(nsteps, 1);

%% Noise
is_noise = true;
noise_K = 1;


%%
for i=1:nsteps
    y(i) = x(1, i);
    x(:, i+1) = simtf(Gs, u(:, i)', dt, x(:, i)) + flip(eye(npoles, 1)*noise_K*randn(1))*is_noise;
end

%%
try
    close(1);
catch er
end
figure(1);
hold on;
stairs(t, y);
for i=1:nzeros
    stairs(t, u(i, :), '--');
end
title('Output and Control Signal');
xlabel('t [s]');
ylabel('Amplitude');
legend('y', 'u');
grid on;
saveas(gcf, FILENAME_TEST + "/output.png");
hold off;

%%
try
    close(2);
catch er
end
figure(2);
subplot(2, 1, 1)
stairs(t, x(1, 1:end-1));
title('State x1');
xlabel('t [s]');
ylabel('Amplitude');
legend('x1');
grid on;

subplot(2, 1, 2)
stairs(t, x(2, 1:end-1));
title('State x2');
xlabel('t [s]');
ylabel('Amplitude');
legend('2');
grid on;
hold on;
saveas(gcf, FILENAME_TEST + "/states.png");


