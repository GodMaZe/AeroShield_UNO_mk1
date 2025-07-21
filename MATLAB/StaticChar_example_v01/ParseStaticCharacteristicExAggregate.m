clear; clc;
close all;


fileslist = dir("data\");

filecounter = 1;

for fi=1:length(fileslist)

    f = fileslist(fi);

    if (f.isdir || ~endsWith(f.name, '.mat'))
        continue;
    end

    filename = split(f.name,'.');
    filename = filename(1);
    filecounter = str2double(filename{1}(2:end));

    load(f.folder + "/" + f.name);
    
    T_step_size = T_step_time; % [s]
    nsamples = T_step_size/T_sample;
    lsamples = max(2 * nsamples/5, 50);
    ulast = u;
    
    nsteps = floor(ulast/STEP_SIZE);
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

    ys{filecounter} = ym;
    ts{filecounter} = tm;
    us{filecounter} = um;

end

totalcount = length(ys);
totalcountnext = length(ys) + 1;

ys{totalcountnext} = zeros(nsteps, 1);
ts{totalcountnext} = zeros(nsteps, 1);
us{totalcountnext} = zeros(nsteps, 1);

for i=1:totalcount
    ys{totalcountnext} = ys{totalcountnext} + ys{i};
    us{totalcountnext} = us{totalcountnext} + us{i};
    ts{totalcountnext} = ts{totalcountnext} + ts{i};
end

ys{totalcountnext} = ys{totalcountnext}/totalcount;
ts{totalcountnext} = ts{totalcountnext}/totalcount;
us{totalcountnext} = us{totalcountnext}/totalcount;

save("aggdata", "ys", "ts", "us", "t_last_step", "totalcount", "totalcountnext", "nsteps", "ulast", "STEP_SIZE");

%% Plot the data

figure(666);
hold on;
xlabel('t [s]');
ylabel('y(t); u(t) [deg; %]');
title('Meranie prevodovej charakteristiky');
subtitle('Porovnanie vstup-vystup')
legend show;
grid on;
ylim([0, max(ys{totalcountnext}) + 10]);
xlim([0, t_last_step + 10]);


figure(999);
hold on;
xlabel('u(t) [%]');
ylabel('y(t) [deg]');
title('Prevodova charakteristika');
% legend show;
grid on;
ylim([0, max(ys{end}) + 10]);
xlim([0, nsteps * STEP_SIZE + 1]);


figure(888);
hold on;
xlabel('t [s]');
ylabel('K [deg/%]');
title('Meranie prevodovej charakteristiky');
subtitle('Podielova');
% legend show;
grid on;
xlim([0, t_last_step + 10]);


for i=1:totalcountnext
 
    namey = "y" + num2str(i);
    nameu = "u" + num2str(i);

    if i == totalcountnext
        namey = "y-mean";
        nameu = "u-mean";
        figure(666);
        plot(ts{i}, ys{i}, '-k', "LineWidth", 2, 'DisplayName', namey);
        plot(ts{i}, us{i}, '--m', "LineWidth", 2, 'DisplayName', nameu);

        figure(999);
        plot(us{i}, ys{i}, '.r', 'MarkerSize', 14, 'DisplayName', "mean");

        figure(888);
        plot(ts{i}, ys{i}./us{i}, '.r', 'MarkerSize', 14, 'DisplayName', 'mean');
    else
        figure(666);
        plot(ts{i}, ys{i}, "LineWidth", 1, 'DisplayName', namey);
        % plot(ts{i}, us{i}, "LineWidth", 1, 'DisplayName', nameu);

        figure(999);
        plot(us{i}, ys{i}, '.k', 'MarkerSize', 8, 'DisplayName', "d" + num2str(i));

        figure(888);
        plot(ts{i}, ys{i}./us{i}, '.k', 'MarkerSize', 8, 'DisplayName', "d" + num2str(i));
    end

    
 
end

hold off;



figure(1666);
hold on
plot(ts{end}, ys{end}, '-k', 'LineWidth', 1.5, 'DisplayName', 'y-mean');
plot(ts{end}, us{end}, '--m', 'LineWidth', 1.5, 'DisplayName', 'u-mean');
xlabel('t [s]');
ylabel('y(t); u(t) [deg; %]');
title('Meranie prevodovej charakteristiky');
subtitle('Porovnanie vstup-vystup')
legend show;
grid on;
ylim([0, max(ys{end}) + 10]);
xlim([0, t_last_step + 10]);
hold off;


figure(1999);
plot(us{end}, ys{end}, '.k', 'MarkerSize', 14);
xlabel('u(t) [%]');
ylabel('y(t) [deg]');
title('Prevodova charakteristika');
% legend show;
grid on;
ylim([0, max(ys{end}) + 10]);
xlim([0, nsteps * STEP_SIZE + 1]);


figure(1888);
plot(ts{end}, ys{end}./us{end}, '.k', 'MarkerSize', 14);
xlabel('t [s]');
ylabel('K [deg/%]');
title('Meranie prevodovej charakteristiky');
subtitle('Podielova');
% legend show;
grid on;
xlim([0, t_last_step + 10]);