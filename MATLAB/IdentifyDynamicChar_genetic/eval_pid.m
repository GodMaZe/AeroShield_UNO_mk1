close all;
clear; clc;

FILENAME="train";

try
    % Load the identified model
    load(FILENAME + "_ident_model");
    % Load the best PID params
    load(FILENAME + "/bestchrom_pid");
    % Load the real-time response (for comparison)
    load(FILENAME + "_pid")
catch er
    fprintf(2, "Error occured while trying to load the necessary files for the evaluation process of the identified PID params. Error: " + er.message)
    rethrow er
end

dt = mean(diff(t));

Gs = Gs20;
Gsd = c2d(Gs, dt);

C = pid(P, I, D, 100, dt);

open_loop = C*Gsd;
closed_loop = feedback(open_loop, 1, -1);

[y, tt] = lsim(closed_loop, w1, t);

eold = 0;
eint = 0;

% x0 = 0;
x0 = [0, 0];

[tx,yx,dy,w,e,de,u,usat] = sim_pid_VIR25(P, I, D, Gs, w1, t, x0);

ALL_MS = [y1, yx, y];

figure(1);
hold on;
plot(t, y1, 'DisplayName', 'y-real', 'LineWidth', 1.5);
plot(t, yx, 'DisplayName', 'y-RK4-c', 'LineWidth', 1.5);
% plot(t, y, 'DisplayName', 'y-matlab', 'LineWidth', 1);
plot(t, w, 'k--', 'DisplayName', 'w', 'LineWidth', 2);
ylim([min(w1) - 0.2, max(ALL_MS, [], "all") + 0.25]);
legend show;
title("Priebeh riadenia");
xlabel("t [s]");
ylabel("y [V]");
grid minor;
hold off;
saveas(gcf, FILENAME + "/eval_y_response.svg");


figure(2);
hold on;
plot(t, u1, 'DisplayName', 'u-real', 'LineWidth', 1.5);
plot(t, u, 'DisplayName', 'u-RK4-c', 'LineWidth', 1.5);
legend show;
title("Akcny zasah");
xlabel("t [s]");
ylabel("u [V]");
grid minor;
hold off
saveas(gcf, FILENAME + "/eval_u_response.svg");