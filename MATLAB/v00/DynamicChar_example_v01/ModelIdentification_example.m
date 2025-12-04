% load("dyn_char.mat");
load("freq_char.mat");


% ----------------------------------
%% Plot the recorded data
% ----------------------------------

t = logsout.t;
y = logsout.y;
u = logsout.u;
r = logsout.r;
e = r - y;
dt = logsout.dt;


figure(333);
hold on;
plot(t, y, '-k', 'LineWidth', 1.5);
plot(t, u, '--b', 'LineWidth', 1.5);
title('Control Response');
legend('y(t)', 'u(t)', "Location", "best");
xlim([0, max(t) + T_sample]);
if min(y) ~= max(y)
    ylim([min(y), max(y)]);
end
xlabel('t [s]');
ylabel('value [deg]/[%]');
grid minor;
hold off;

% ----------------------------------
% ----------------------------------

% ----------------------------------
%% Process the data
% ----------------------------------

Ts = mean(diff(t));
Fs = 1/Ts;

idata = iddata(y, u, Ts);

% ----------------------------------
% ----------------------------------

% ----------------------------------
%% Estimate a SS and TF model
% ----------------------------------

tfmodel = tfest(idata, 2, 1);

ssmodel = ssest(idata, 2);

% ----------------------------------
% ----------------------------------


% ----------------------------------
%% Compare the models with the real data
% ----------------------------------

figure(2);
compare(idata, tfmodel, ssmodel);

% ----------------------------------
% ----------------------------------

% ----------------------------------
%% Fourier analysis
% ----------------------------------
[b, a] = butter(4, 0.05,"low");
yfilt = filter(b,a,y);

figure(666);
plot(t, y, t, yfilt);
legend("y","yfilt");
title("Compare real vs filtered");
xlabel("t [s]");
ylabel("$\varphi [^\circ]$", "Interpreter","latex");
grid minor;

yf = fft(yfilt);
L = length(t);
samples = 0:L-1;

% freqs = (-L/2:L/2-1)*Fs/L;
freqs = (0:L-1)*Fs/L;


figure(111);
semilogx(freqs, 20*log10(abs(yf)));
title("Complex Magnitude of fft Spectrum");
xlabel("f (Hz)");
ylabel("|fft(x)|");
grid minor;

phi = rad2deg(angle(yf));
phifilt = filter(b, a, phi);

figure(222);
semilogx(freqs, phifilt);
title("Phase Spectrum of y(t)");
xlabel("f [Hz]");
ylabel("$\Phi \left[^\circ\right]$", "Interpreter","latex");
xlim([0, max(freqs)]);
grid minor;
% ----------------------------------
% ----------------------------------

return

% ----------------------------------
%% Fourier test
% ----------------------------------

Fs = 80; % Hz
T = 1/Fs; % s
L = 81;
tsamples = (0:L-1);
t = tsamples*T;

F1 = 5; % Hz
F2 = 10; % Hz
F3 = 31; % Hz

X = 3*cos(2*pi*F1*t) + 2*cos(2*pi*F2*t) + 5*sin(2*pi*F3*t);

plot(t,X)
title("Signal superposition in time domain")
xlabel("t (ms)")
ylabel("X(t)")

Y = fft(X);

f = Fs * (0:(L-1)/2)/L;
P2 = abs(Y/L);
P1 = P2(1:(L+1)/2);
P1(2:end) = 2*P1(2:end);

P1filt = filter(b, a, P1);

plot(f,P1filt,"-o") 
title("Single-Sided Spectrum of Original Signal")
xlabel("f (Hz)")
ylabel("|P1(f)|")

% ----------------------------------
% ----------------------------------
