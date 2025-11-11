clear;
close all;
clc;
%% Load the data
DATADIR = "./data";

if ~exist(DATADIR, "dir")
    mkdir(DATADIR);
    fprintf("Created the data dir... Put the files to identify into the data folder (train.mat, val.mat files required).\n");
    return;
end

FILENAME_TRAIN = "train";
FILENAME_VAL = "val";

load(DATADIR + "/" + FILENAME_TRAIN);

if ~exist(FILENAME_TRAIN, "dir")
    mkdir(FILENAME_TRAIN);
end

if ~exist(FILENAME_VAL, "dir")
    mkdir(FILENAME_VAL);
end

%% Plot the data
t = logsout.tp';

tind = find(t >= SYNC_TIME, 1);

mask = tind:numel(t);

t = t(mask);
t = t - t(1);
dt = mean(diff(t));
y = logsout.y(mask)';
y = y - mean(logsout.y(tind-30:tind));
u = logsout.u(mask)';
u = u - logsout.u(tind-10);

figure(1);
plot(t, y, t, u, '--k');
legend("y","u", location="south east");
xlabel("t [s]");
ylabel("value [-]");
title("Plot real data");
grid minor;
saveas(gcf, FILENAME_TRAIN + "/ident_measurement.svg");

%% Identify the model
idata = iddata(y, u, dt);

Gs20 = tfest(idata, 2, 0);
Gs21 = tfest(idata, 3, 0);
Gsss = ssest(idata, 3);

%% Simulate the model

t = 0:dt:t(end);

ym20 = lsim(Gs20, u, t);
ym21 = lsim(Gs21, u, t);
ymss = lsim(Gsss, u, t);

%% Plot the simulation and real
% MSE20 = mean((ym20 - y).^2);
% MSE21 = mean((ym21 - y).^2);

figure(2);
plot(t, y, t, ym20, t, ym21, t, ymss, t, u);
legend("y","ym20","ym30","ymss","u", location="south east");
title("Model/Real response");
subtitle("MSE20: " + num2str(Gs20.Report.Fit.MSE) + " | MSE30: " + num2str(Gs21.Report.Fit.MSE) + " " + ...
    " MSESS: " + num2str(Gsss.Report.Fit.MSE));
xlabel("t [s]");
ylabel("value [-]");
grid minor;
saveas(gcf, FILENAME_TRAIN + "/sys_identify.svg");

%% Save the model
save(FILENAME_TRAIN + "_ident");
save(FILENAME_TRAIN + "_ident_model", "Gs21", "Gs20", "Gsss");

%% Validation
load(DATADIR + "/" + FILENAME_VAL);

t = logsout.tp';

tind = find(t >= SYNC_TIME, 1);

mask = tind:numel(t);

t = t(mask);
t = t - t(1);
dt = mean(diff(t));
y = logsout.y(mask)';
y = y - mean(logsout.y(tind-30:tind));
u = logsout.u(mask)';
u = u - logsout.u(tind-10);

t = 0:dt:t(end);

ym20 = lsim(Gs20, u, t);
ym21 = lsim(Gs21, u, t);
ymss = lsim(Gsss, u, t);

MSE20 = mean((ym20 - y).^2);
MSE30 = mean((ym21 - y).^2);
MSESS = mean((ymss - y).^2);

figure(3);
plot(t, y, t, ym20, t, ym21, t, ymss, t, u);
legend("y","ym20","ym30","ymss","u", location="south east");
title("Validation Model/Real response");
subtitle("MSE20: " + num2str(MSE20) + " | MSE30: " + num2str(MSE30) + " | MSESS: " + num2str(MSESS));
xlabel("t [s]");
ylabel("value [-]");
grid minor;
saveas(gcf, FILENAME_VAL + "/sys_ident_val.svg");