clear; clc;
close all;

load("data/s2.mat");

T_step_size = T_step_time; % [s]
nsamples = T_step_size/T_sample;
lsamples = max(2 * nsamples/5, 50);
ulast = u;

nsteps = floor(ulast/STEP_SIZE - 1);
t_last_step = 0;

y = cell(nsteps, 1);
t = cell(nsteps, 1);
u = cell(nsteps, 1);
current = cell(nsteps, 1);

istep = 1;
ilast = 0;
ulast = 0;

% Sort the measured data into cell arrays per each different control input
for i=1:length(static_char.t)
    ti = static_char.t(i);
    ui = static_char.u(i);

    if ui > ulast
        ilast = i - 1;
        istep = istep + 1;
        t_last_step = ti;
        ulast = ulast + STEP_SIZE;
        % In case the measured data are longer than the wanted input window
        % (for example only taking the input values from 0-10, but the
        % experiment was halted after the input has already changed to 11,
        % thus containing more measurements than needed)
        if istep > nsteps
            break;
        end
    end

    y{istep, 1}(i - ilast) = static_char.y(i);
    t{istep, 1}(i - ilast) = ti;
    u{istep, 1}(i - ilast) = ui;
    current{istep, 1}(i - ilast) = static_char.i(i);
end


ym = nan(nsteps, 1);
tm = nan(nsteps, 1);
um = nan(nsteps, 1);
im = nan(nsteps, 1);

for i=1:nsteps
    ym(i) = mean(y{i}(end-lsamples:end));
    tm(i) = mean(t{i}(end-lsamples:end));
    um(i) = mean(u{i}(end-lsamples:end));
    im(i) = mean(current{i}(end-lsamples:end));
end

figure(666);
plot(tm, ym, '-k', tm, um, '-m', "LineWidth", 1);
xlabel('t [s]');
ylabel('y(t); u(t) [deg; V]');
title('Meranie prevodovej charakteristiky');
subtitle('Porovnanie vstup-vystup')
legend('y','u', "Location", "northwest");
grid on;
ylim([0, max(y{end}) + 10]);
xlim([0, t_last_step + 10]);

figure(999);
plot(um, ym, 'xk');
xlabel('u(t) [V]');
ylabel('y(t) [deg]');
title('Prevodova charakteristika');
legend('y', "Location", "northwest");
grid on;
ylim([0, max(y{end}) + 10]);
xlim([0, nsteps * STEP_SIZE + 1]);

figure(888);
plot(um, ym./um, 'xk');
xlabel('u [%]');
ylabel('K [deg/V]');
title('Meranie prevodovej charakteristiky');
subtitle('Podielova');
legend('y', "Location", "northwest");
grid on;
xlim([0, max(um) + 1]);

figure(1111);
plot(tm, im, '-k', "LineWidth", 2);
xlabel('t [s]');
ylabel('i(t) [A]');
title('Meranie prevodovej charakteristiky');
subtitle('Prud')
legend('i', "Location", "northwest");
grid on;
ylim([min(current{end}) - min(current{end})/10, max(current{end}) + max(current{end})/10]);
xlim([0, t_last_step + 10]);

hnsteps = ceil(nsteps/2);

% Visualize the points used for estimating the static char
figure(1); plot(t{hnsteps},y{hnsteps},'k','LineWidth',1); hold on; plot(t{hnsteps}(end-lsamples:end), y{hnsteps}(end-lsamples:end), 'r', 'LineWidth', 3); legend('y','yt');

% Static model

TAKE_U_UPTO = 69;

ltake = find(um > TAKE_U_UPTO, 1);

if isempty(ltake)
    ltake = length(um);
else
    ltake = ltake - 1;
end

umNonLin = um(1:ltake);
ymNonLin = ym(1:ltake);

polyKoef = polyfit(umNonLin, ymNonLin, 3);
yhat = polyval(polyKoef, umNonLin);

scoefs = rmmissing(split(num2str(polyKoef), ' '));
ncoefs = length(scoefs);

smodel = "";

for i=1:ncoefs
    if i == 1
        smodel = scoefs(i) + "u^" + num2str(ncoefs - i);
    elseif i == ncoefs
        smodel = smodel + " + "  + scoefs(i);
    else
        smodel = smodel + " + " + scoefs(i) + "u^" + num2str(ncoefs - i);
    end
end

sqerr = (abs(yhat - ymNonLin).^2);

sse = sum(sqerr);

figure(2);
hold on;
plot(umNonLin, ymNonLin, '+r');
plot(umNonLin, yhat, '.b');
xlabel('u [%]');
ylabel('y [deg]');
title('Static model');
subtitle("SSE = " + num2str(sse));
errorbar(umNonLin, ymNonLin, std(sqerr), '.r');
legend('y', 'yhat', 'Location', 'northwest');
grid minor;

udense = 0:0.1:TAKE_U_UPTO;
yhat = polyval(polyKoef, udense);

figure(3);
hold on;
plot(umNonLin, ymNonLin, '+r');
plot(udense, yhat, '.b');
xlabel('u [%]');
ylabel('y [deg]');
title('Static model');
subtitle("yh = " + smodel);
legend('y', 'yhat', 'Location', 'northwest');
grid minor;



fprintf("Static model is: yhat = " + smodel + "\n");

