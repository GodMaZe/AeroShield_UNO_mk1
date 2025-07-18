clear; clc;
close all;

load("data/s1.mat");

T_step_size = T_step_time; % [s]
nsamples = T_step_size/T_sample;
lsamples = max(2 * nsamples/5, 50);
ulast = u;

nsteps = floor(ulast/STEP_SIZE - 1);
t_last_step = 0;

y = cell(nsteps, 1);
t = cell(nsteps, 1);
u = cell(nsteps, 1);

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
    
end


ym = nan(nsteps, 1);
tm = nan(nsteps, 1);
um = nan(nsteps, 1);

for i=1:nsteps
    ym(i) = mean(y{i}(end-lsamples:end));
    tm(i) = mean(t{i}(end-lsamples:end));
    um(i) = mean(u{i}(end-lsamples:end));
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
plot(tm, ym./um, 'xk');
xlabel('t [s]');
ylabel('K [deg/V]');
title('Meranie prevodovej charakteristiky');
subtitle('Podielova');
legend('y', "Location", "northwest");
grid on;
xlim([0, t_last_step + 10]);