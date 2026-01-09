close all; clear;
clc;

addpath("./misc");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "OPT_measurement";

if ~exist(DDIR, "dir")
    fprintf("Creating the default data repository folder, for saving the measurements...\n");
    mkdir(DDIR);
end

DateString = convertCharsToStrings(datestr(datetime('now'), "yyyy_mm_dd_HH_MM_ss"));

% File paths for the csv and mat files.
FILEPATH = getfilename(DDIR, FILENAME, DateString);
FILEPATH_MAT = getfilename(DDIR, FILENAME, DateString, 'mat');

% The names of the parameters to write into the file
OUTPUT_NAMES = ["t", "tp", "y", "u", "pot", "dt", "step", "xhat_1", "xhat_2", "xhat_3", "z"];

%% Declare all the necessary variables
Tstop = 60;

Ts = 0.05;

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

global LOG_T LOG_TP LOG_POT LOG_Y LOG_U LOG_X_HAT_1 LOG_X_HAT_2 LOG_X_HAT_3 LOG_Z LOG_DT LOG_STEP;

% Logging vectors
LOG_T = [];
LOG_TP = [];
LOG_POT = [];
LOG_Y = [];
LOG_U = [];
LOG_X_HAT_1 = [];
LOG_X_HAT_2 = [];
LOG_X_HAT_3 = [];
LOG_Z = [];
LOG_DT = [];
LOG_STEP = [];

function plotdatarealtime()
    global LOG_T LOG_Y LOG_U LOG_X_HAT_2;
    persistent hy hr hu;
    % ----------------------------------
    % Plot the measured data in real time
    % ----------------------------------
    try
        if isempty(hy) || isempty(hr) || isempty(hu)
            f = figure("Name","Plot-RealTime");
            ax = axes(f);
            hold on;
            % hy = plot(ax, nan, nan, '.k');
            % hr = plot(ax, nan, nan, '.r');
            % hu = plot(ax, nan, nan, '.b');
            hy = stairs(ax, nan, nan);
            hr = stairs(ax, nan, nan);
            hu = stairs(ax, nan, nan);
            grid minor;
            title("Real-Time System Response");
            xlabel("t [s]");
            ylabel("$\varphi [^\circ]$", "Interpreter","latex");
            legend(ax, "y","xhat_{2}", "u", 'Location', 'southeast');
            
        end
       
        % plot(plot_t, plot_sig_3,'.b', plot_t, plot_sig_2,'.r', plot_t,
        % plot_sig_1,'.k')
        % print(timer_t(1));
        nsteps = numel(LOG_Y);
        last_n_points = max(1, nsteps - 300);

        mask = last_n_points:nsteps;
        t = LOG_T(mask);

        set(hy, 'YData', LOG_Y(mask), 'XData', t);
        set(hr, 'YData', LOG_X_HAT_2(mask), 'XData', t);
        set(hu, 'YData', LOG_U(mask), 'XData', t);
        drawnow limitrate nocallbacks;
    catch err
       fprintf(2, "Plot thread: " + err.message + "\n");
    end
    % ----------------------------------
    % ----------------------------------
end




try
    timerplotrealtime = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotdatarealtime());
    start(timerplotrealtime);
    
    % Open the CSV file for writing
    if(exist("dfile_handle", "var"))
        fclose(dfile_handle);
        clear dfile_handle;
    end
    
    dfile_handle = fopen(FILEPATH,'w');
    fprintf(dfile_handle, join(OUTPUT_NAMES,",") + "\n");

    % Open the serial port for communication with the Arduino (system)
    if(exist("scon", "var"))
        scon.flush("input");
        clear scon;
    end

    scon = serialport("COM7", 250000, "Timeout", 5);
    
    % Read the initial conditions
    sline = str2num(readline(scon));
    disp(sline);

    init_plant_time = sline(1);
    plant_potentiometer = sline(2);
    plant_output = sline(3);
    plant_input = sline(4);
    plant_x_hat_1 = sline(5);
    plant_x_hat_2 = sline(6);
    plant_x_hat_3 = sline(7);
    plant_z = sline(8);

    time_start = datetime("now");
    time_curr = time_start;
    time_last = time_curr;

    plant_time_init = -1;
    plant_time = 0;
    step = 0;
    is_init = false;
    
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if step > 0 && time_delta < Ts
            continue;
        end
        
        % Wait for the system to send a data message
        sline = str2num(readline(scon));

        if plant_time_init < 0
            plant_time_init = sline(1);
        end

        plant_time = (sline(1) - plant_time_init)/1e6;
        plant_potentiometer = sline(2);
        plant_output = sline(3);
        plant_input = sline(4);
        plant_x_hat_1 = sline(5);
        plant_x_hat_2 = sline(6);
        plant_x_hat_3 = sline(7);
        plant_z = sline(8);

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, time_delta, plant_x_hat_1, plant_x_hat_2, plant_x_hat_3, plant_z];
        writenum2file(dfile_handle, data, mod(step, 10)==0, Ts, time_delta);

        LOG_T = [LOG_T, time_elapsed];
        LOG_TP = [LOG_TP, plant_time];
        LOG_Y = [LOG_Y, plant_output];
        LOG_U = [LOG_U, plant_input];
        LOG_POT = [LOG_POT, plant_potentiometer];
        LOG_X_HAT_1 = [LOG_X_HAT_1, plant_x_hat_1];
        LOG_X_HAT_2 = [LOG_X_HAT_2, plant_x_hat_2];
        LOG_X_HAT_3 = [LOG_X_HAT_3, plant_x_hat_3];
        LOG_Z = [LOG_Z, plant_z];
        LOG_DT = [LOG_DT, time_delta];
        LOG_STEP = [LOG_STEP, step];

        step = step + 1;
        time_last = time_curr;

        if plant_time >= Tstop || plant_output >= Ystop
            break;
        end
    end

catch er
    % Send a final command and close the serial port
    if exist("scon", "var")
        write(scon, 0.0, 'single');
        clear scon;
    end
    if exist("dfile_handle", "var")
        fclose(dfile_handle);
        clear dfile_handle;
    end
    for tim=timerfindall
        stop(tim);
        delete(tim);
    end
    rethrow(er);
end

%% close conns
if exist("scon", "var")
    write(scon, 0.0, 'single');
    clear scon;
end
if exist("dfile_handle", "var")
    fclose(dfile_handle);
    clear dfile_handle;
end
for tim=timerfindall
    stop(tim);
    delete(tim);
end

%% Save the measurement
nsteps = LOG_STEP(end);
logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_DT, LOG_STEP, LOG_X_HAT_1, LOG_X_HAT_2, LOG_X_HAT_3, LOG_Z, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================
figure(999);
style='-k';

subplot(2,1,1)
hold on;
stairs(LOG_TP, LOG_X_HAT_2,"LineWidth",1.5);
stairs(LOG_TP, LOG_Y,'LineWidth',1.5);
% scatter(LOG_TP, LOG_Y, '.k');
xlabel('k'); ylabel('\phi(k)'); grid on
% xlim([0,max(LOG_STEP)]);
legend("yhat", "y");
hold of;

subplot(2,1,2)
stairs(LOG_TP, LOG_U,style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
% xlim([0,max(LOG_STEP)]);

set(gcf,'position',[200,400,650,400]);

figure(123); clf;

subplot(3, 1, 1);
hold on;
stairs(LOG_TP, LOG_X_HAT_1);
hold off;
ylabel("uhat [%PWM]");
title("X HAT_1");
grid minor;
grid on;

subplot(3, 1, 2);
hold on;
stairs(LOG_TP, LOG_X_HAT_2);
stairs(LOG_TP, LOG_Y);
plot(LOG_TP, LOG_POT);
hold off;
ylabel("yhat [deg]");
legend("yhat", "y", "yref");
title("X HAT_2");
grid minor;
grid on;

subplot(3, 1, 3);
hold on;
stairs(LOG_TP, LOG_X_HAT_3);
hold off;
ylabel("vhat [deg/s]");
xlabel("t [s]");
title("X HAT_3");
grid minor;
grid on;
hold off;

figure(111); clf;
stairs(LOG_TP, LOG_Z);
xlabel("t [s]");
ylabel("z [-]");
title("LQR integrator");
grid minor;
grid on;
