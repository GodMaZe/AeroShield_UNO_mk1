clear;
close all; clc;

if ~exist("aggdata.mat", "file")
    fprintf(2, "No aggregate data file found. Make sure, the root directory contains the aggdata.mat file with the correct data.");
    return;
end


load("aggdata.mat");

%% Visualize the aggregate data


figure(111);
hold on;
h1 = plot([ts{:,end}], [ys{:,end}], '-k', 'LineWidth', 1.5, 'DisplayName', 'y');
h2 = plot([ts{:,end}], [us{:,end}], '-r', 'LineWidth', 1.5, 'DisplayName', 'u');
% plot(t(tcindx), u(tcindx), 'xr', 'MarkerSize', 15);
legend([h1(1), h2(1)], {'y', 'u'});
title("Prechodove charakteristiky");
subtitle("Spriemerovane");
grid minor;
hold off;

figure(222);
hold on;
h1 = plot([ts{:,1:STEP_REPS}], [ys{:,1:STEP_REPS}], '-k', 'DisplayName', 'y');
h2 = plot([ts{:,1}], [us{:,1}], '-r', 'DisplayName', 'u');
title("Prechodove charakteristiky");
subtitle("Agregovane");
legend([h1(1), h2(1)], {'y', 'u'});
grid minor;
hold off;

%% Separate step-ups from step-downs
Ts = mean(diff([ts{end, end}]));
du = gradient(us{end, end} - median(us{end, end}), Ts);
duidx = find(du ~= 0);
duidx(1) = 1;

ldu = length(duidx);

min_n_step = floor(nelem/(nprocessorder + 1)/2);
cstep = 1;

for i=2:ldu
    fidx = duidx(i);
    if fidx - cstep <= min_n_step
        duidx(i) = nan;
    else
        cstep = fidx;
    end
    
end

duidx = rmmissing(duidx);

STEP_TIME_TO_PROCESS = STEP_TIME/2;

nitems = floor(STEP_TIME_TO_PROCESS/Ts);

% parsed data
ndata = length(ts);
py = cell(nops, ndata, nprocessorder);
pu = cell(nops, ndata, nprocessorder);
pt = cell(nprocessorder, 4); % All the times are identical, thus use a cell with nprocessorder time vectors

for i = 1:nprocessorder
    tts = ts{end, end}; % Using the averaged time (all the same, because of the time interpolation)
    cidx = duidx(i);
    idxselect = cidx:cidx+nitems;
    inittts = tts(cidx);
    pt{i, 1} = tts(idxselect) - inittts;
    pt{i, 2} = idxselect;
    pt{i, 3} = inittts;
    pt{i, 4} = du(cidx) > 0; % Check whether step-up or down. UP = 1 (true); DOWN = 0 (false)
end

for h = 1:nops
    for i = 1:ndata
        for x = 1:nprocessorder
            py{h, i, x} = ys{h, i}(pt{x, 2});
            pu{h, i, x} = us{h, i}(pt{x, 2});
        end
    end
end


%% Plot parsed data

upbidx = 1;
procorder = 4;

Y = [py{upbidx, :, procorder}];
U = [pu{upbidx, :, procorder}];
upb = median([us{upbidx, :}]);
tfin = [pt{1, 1}];

Yf = (Y - Y(1,:))/STEP_SIZE;
Uf = (U - upb)/STEP_SIZE;

figure(333);
hold on;
h1 = plot(tfin, Yf, '-k', 'DisplayName', 'y');
h2 = plot(tfin, Uf, '-r', 'DisplayName', 'u');
title("Prechodove charakteristiky");
subtitle("Agregovane - STEP UP");
legend([h1(1), h2(1)], {'y', 'u'});
grid minor;
hold off;


%% Identify the transfer function

y = [ys{1, end}];
u = [us{1, end}];

idata = iddata(Y(:,end), U(:, end), T_sample);

Gs = tfest(idata, 2);
Gs2 = arx(idata, [2 1 1]);
Gs3 = armax(idata, [2 1 1 1]);

%% Plot the models and data
yfin = Yf(:, end);
figure(9999);
hold on;
plot(tfin, yfin, 'LineWidth', 1.5);
[stepy1, stept1] = step(Gs, tfin);
[stepy2, stept2] = step(Gs2, tfin);
[stepy3, stept3] = step(Gs3, tfin);
plot(stept1, stepy1, stept2, stepy2, stept3, stepy3, 'LineWidth', 1.5);
grid minor;
e1 = sum((stepy1 - yfin).^2);
e2 = sum((stepy2 - yfin).^2);
e3 = sum((stepy3 - yfin).^2);
legend("G1: " + num2str(e1), "G2: " + num2str(e2), "G3: " + num2str(e3), "Data");
title("Step response");
subtitle("comparison");
xlabel("t [s]");
ylabel("y (deg)");
