close all;
clear; clc;

load("aggdata.mat");

%% Plot the data

figure(666);
hold on;
xlabel('t [s]');
ylabel('y(t); u(t) [deg; %]');
title('Meranie prevodovej charakteristiky');
subtitle('Porovnanie vstup-vystup')
legend('Location', 'northwest');
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
xlabel('u [%]');
ylabel('K [deg/%]');
title('Meranie prevodovej charakteristiky');
subtitle('Podielova');
% legend show;
grid on;
xlim([0, ulast]);

figure(777);
hold on;
xlabel('u [%]');
ylabel('K [deg/%]');
title('Meranie prevodovej charakteristiky');
subtitle('Zosilnenie');
% legend show;
grid on;
xlim([0, ulast]);


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
        plot(us{i}, ys{i}./us{i}, '.r', 'MarkerSize', 14, 'DisplayName', 'mean');
        
        figure(777);
        plot(us{i}, gradient(ys{i}), '-r', 'MarkerSize', 10, 'DisplayName', 'mean');
    else
        figure(666);
        plot(ts{i}, ys{i}, "LineWidth", 1, 'DisplayName', namey);
        % plot(ts{i}, us{i}, "LineWidth", 1, 'DisplayName', nameu);

        figure(999);
        plot(us{i}, ys{i}, '.k', 'MarkerSize', 8, 'DisplayName', "d" + num2str(i));

        figure(888);
        plot(us{i}, ys{i}./us{i}, '.k', 'MarkerSize', 8, 'DisplayName', "d" + num2str(i));

        figure(777);
        plot(us{i}, gradient(ys{i}), '.k', 'MarkerSize', 8, 'DisplayName', "d" + num2str(i));
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
legend('Location', 'northwest');
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
plot(us{end}, ys{end}./us{end}, '.k', 'MarkerSize', 14);
xlabel('u [%]');
ylabel('K [deg/%]');
title('Meranie prevodovej charakteristiky');
subtitle('Podielova');
% legend show;
grid on;
xlim([0, ulast]);

figure(1777);
plot(us{end}, gradient(ys{end}), '.k', 'MarkerSize', 14);
xlabel('u [%]');
ylabel('K [deg/%]');
title('Meranie prevodovej charakteristiky');
subtitle('Zosilnenie');
% legend show;
grid on;
xlim([0, ulast]);

% Static model

ym = ys{end};
um = us{end};

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

figure(1);
hold on;
plot(umNonLin, ymNonLin, '+r');
plot(udense, yhat, '.b');
xlabel('u [%]');
ylabel('y [deg]');
title('Static model');
legend('y', 'yhat', 'Location', 'northwest');
grid minor;

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

fprintf("Static model is: yhat = " + smodel + "\n");