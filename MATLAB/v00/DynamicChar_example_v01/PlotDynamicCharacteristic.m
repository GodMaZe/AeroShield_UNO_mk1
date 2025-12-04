clear;
close all; clc;

load("./data/d5.mat");


t = dyn_char.t;
u = dyn_char.u;
y = dyn_char.y;

tp = dyn_char.tp;

% Fix arduino long overflow
while true
    [tpv,tpidx] = max(tp);
    if tpidx < length(tp)
        tp(tpidx+1:end) = tp(tpidx+1:end) + tpv;
    else
        break;
    end
end

Ts = mean(diff(tp))/1e6; % Get the average sampling period (org. time is in [ms])

du = gradient(u, Ts);

ldata = length(t);

% [b,a] = butter(10, 1/10, "low");
% 
% y = filter(b, a, y);

% nops = 5;

ys = cell(nops, STEP_REPS + 1);
ts = cell(nops, STEP_REPS + 1);
us = cell(nops, STEP_REPS + 1);


uindx = find(abs(du) > STEP_SIZE/Ts/4);
tindx = find(t < T_sample, nops * (STEP_REPS + 1));

tcindx = tindx(1:STEP_REPS + 1:end);

uindx = uindx(2:nprocessorder*2:end);


% tindx = setdiff(tindx, tcindx);

figure(6666); hold on; plot(tp/1e6, u, 'LineWidth', 1); plot(tp(uindx)/1e6, u(uindx), 'or'); plot(tp(tcindx)/1e6, u(tcindx), 'om'); grid on;
figure(6669); hold on; plot(tp/1e6, du, 'LineWidth', 1); plot(tp(uindx)/1e6, du(uindx), 'or'); plot(tp(tcindx)/1e6, du(tcindx), 'om'); grid on;
% return;


T_COUNTER = 2;

istep = 1;
irep = 1;

for i=1:length(uindx)
    otcounter = T_COUNTER;
    cidx = uindx(i);
    if i + 1 > length(uindx)
        getter = cidx:ldata;
    elseif mod(i, STEP_REPS) == 0
        getter = cidx:tcindx(T_COUNTER) - 1;
        T_COUNTER = T_COUNTER + 1;
    else
        getter = cidx:uindx(i + 1) - 1;
    end

    ys{istep, irep} = y(getter);
    us{istep, irep} = u(getter);
    ts{istep, irep} = t(getter);
    if ts{istep, irep}(end) < ts{istep, irep}(end - 1)
        ts{istep, irep}(end) = t(end-1) + min(T_sample, max(0, ts{istep, irep}(end - 1)));
    end
    ts{istep, irep}(:) = ts{istep, irep}(:) - ts{istep, irep}(1);

    if T_COUNTER > otcounter
        istep = istep + 1;
        irep = 0;
    end

    irep = irep + 1;
end


% return;
 
% Fix missing data due to increased sampling time for a measurement
nelem = (STEP_TIME * nprocessorder)/T_sample + 1;

tt = (0:T_sample:(nprocessorder * STEP_TIME))';

ut = ones(nprocessorder, STEP_TIME/T_sample) .* STEP_PROCESS_ORDER' * STEP_SIZE;
ut = [STEP_PROCESS_ORDER(1) * STEP_SIZE; reshape(ut.', 1, [])'];
%% Interpolation Loopback

% Either use linear extrapolation for the few last missing points or cut
% all the vectors to the smallest size
for i=1:nops
    for x=1:STEP_REPS
        lint = length(ts{i, x});
        nmvals = nelem - lint;
        if nmvals < nelem/5.0
            % used for comparing the interpolated and real data.
            % ys{i, x+20} = ys{i, x};
            % us{i, x+20} = us{i, x};
            % ts{i, x+20} = ts{i, x};

            ys{i, x} = interp1(ts{i, x}, ys{i,x}, tt, "linear","extrap");
            us{i, x} = ut + median(us{i,x}); %interp1(ts{i, x}, us{i,x}, tt, "previous","extrap");
            ts{i, x} = tt;
        elseif nmvals == 0
            continue;
        else
            ys{i, x} = [];
            us{i, x} = [];
            ts{i, x} = [];
            fprintf(2, "Cannot interpolate values in the provided timeframe because more than 4/5 of the data is missing! Cell index: {%d, %d}\n", i, x);
        end
    end
end



for i=1:nops
    ys{i, STEP_REPS + 1} = mean([ys{i, 1:STEP_REPS}], 2, "omitmissing");
    us{i, STEP_REPS + 1} = mean([us{i, 1:STEP_REPS}], 2, "omitmissing");
    ts{i, STEP_REPS + 1} = mean([ts{i, 1:STEP_REPS}], 2, "omitmissing");
end

% Use this plot when visualizing the interpolated values compared to the
% real measured data.
% figure(1); plot(tt, ys{1,1}, 'k'); hold on; plot(ts{1,21},ys{1,21},'.r');

save("aggdata", "ys", "ts", "us", "nelem", "STEP_REPS", "STEP_TIME", "STEP_STAB_TIME", "STEP_STAB_DU", "STEP_SIZE", "STEP_PROCESS_ORDER", "nprocessorder", "nops", "T_sample", "T_COUNTER", "T_start", "T_sample");


figure(111);
h1 = plot([ts{:,end}], [ys{:,end}], '-k', 'LineWidth', 1.5, 'DisplayName', 'y');
hold on;
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