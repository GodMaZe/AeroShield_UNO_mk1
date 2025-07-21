load("aggdata.mat");

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