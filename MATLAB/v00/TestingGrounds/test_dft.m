clear; close all; clc;

addpath("../../misc/functions")

Ts = 0.001;
Tstop = 1;
t = 0:Ts:Tstop;

freq = [1 5 10 50];
A = [5 1 2 6];

x = A * sin(2*pi*freq'*t);

Gs = tf([5], [1, 1, 5]);

[x, t] = step(Gs);

x = x';

figure(1); clf;
plot(t, x);
xlabel('Time (s)');
ylabel('Amplitude');
title('Sum of Sinusoidal Signals');
grid minor;
grid on;


% Do the dft

X = dft(x);

N = numel(x) - 1;
n = 0:N;
T = N/(1/Ts);
f = n/T;

mag = (abs(X));
phase = atan2(imag(X), real(X));

% Plot the magnitude and phase of the DFT
figure(2); clf;
subplot(2, 1, 1);
semilogx(f, mag);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Magnitude Spectrum');
grid on;

subplot(2, 1, 2);
semilogx(f, rad2deg(phase));
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Phase Spectrum');
grid on;