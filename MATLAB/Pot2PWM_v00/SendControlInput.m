close all; clear;
clc;

addpath("./misc");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "measurement";

if ~exist(DDIR, "dir")
    fprintf("Creating the default data repository folder, for saving the measurements...\n");
    mkdir(DDIR);
end

DateString = convertCharsToStrings(datestr(datetime('now'), "yyyy_mm_dd_HH_MM_ss"));

% File paths for the csv and mat files.
FILEPATH = getfilename(DDIR, FILENAME, DateString);
FILEPATH_MAT = getfilename(DDIR, FILENAME, DateString, 'mat');

% The names of the parameters to write into the file
OUTPUT_NAMES = ["t", "tp", "y", "u", "pot", "dtp", "dt", "step"];

%% Declare all the necessary variables
Tstop = 60;

Ts = 0.05;

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

global LOG_T LOG_TP LOG_POT LOG_Y LOG_U LOG_DTP LOG_DT LOG_STEP;

% Logging vectors
LOG_T = [];
LOG_TP = [];
LOG_POT = [];
LOG_Y = [];
LOG_U = [];
LOG_DTP = [];
LOG_DT = [];
LOG_STEP = [];

function plotdatarealtime()
    global LOG_T LOG_Y LOG_U LOG_POT;
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
            legend(ax, "y","pot", "u", 'Location', 'southeast');
            
        end
       
        % plot(plot_t, plot_sig_3,'.b', plot_t, plot_sig_2,'.r', plot_t,
        % plot_sig_1,'.k')
        % print(timer_t(1));
        nsteps = numel(LOG_Y);
        last_n_points = max(1, nsteps - 300);

        mask = last_n_points:nsteps;
        t = LOG_T(mask);

        set(hy, 'YData', LOG_Y(mask), 'XData', t);
        set(hr, 'YData', LOG_POT(mask), 'XData', t);
        set(hu, 'YData', LOG_U(mask), 'XData', t);
        drawnow limitrate nocallbacks;
    catch err
       fprintf(2, "Plot thread: " + err.message + "\n");
    end
    % ----------------------------------
    % ----------------------------------
end

timerplotrealtime = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotdatarealtime());
start(timerplotrealtime);


try
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

    scon = serialport("COM4", 9600, "Timeout", 5);
    
    sline = "";

    while(~contains(sline, "MCU"))
        % disp(sline);
        sline = readline(scon);
    end

    disp(sline);

    % Read the initial conditions
    sline = str2num(readline(scon));
    disp(sline);

    init_plant_time = sline(1);
    plant_potentiometer = sline(2);
    plant_output = sline(3);
    plant_input = sline(4);
    plant_dt = sline(5);

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

        if time_delta < Ts && is_init
            continue;
        end

        is_init = true;

        % Communication sync., it has no other effect.
        write(scon, 0.0, "single");
        
        % Wait for the system to send a data message
        sline = str2num(readline(scon));

        if plant_time_init < 0
            plant_time_init = sline(1);
        end

        plant_time = sline(1) - plant_time_init;
        plant_potentiometer = sline(2);
        plant_output = sline(3);
        plant_input = sline(4);
        plant_dt = sline(5);

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta];
        writenum2file(dfile_handle, data, mod(step, 1)==0, Ts, time_delta);

        LOG_T = [LOG_T, time_elapsed];
        LOG_TP = [LOG_TP, plant_time];
        LOG_Y = [LOG_Y, plant_output];
        LOG_U = [LOG_U, plant_input];
        LOG_POT = [LOG_POT, plant_potentiometer];
        LOG_DTP = [LOG_DTP, plant_dt];
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
logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_DTP, LOG_DT, LOG_STEP, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================
figure(999);
style='-k';

subplot(2,1,1)
hold on;
plot(LOG_TP, LOG_POT,"--k","LineWidth",1.5);
stairs(LOG_TP, LOG_Y,'LineWidth',1.5);
% scatter(LOG_TP, LOG_Y, '.k');
xlabel('k'); ylabel('\phi(k)'); grid on
% xlim([0,max(LOG_STEP)]);
hold of;

subplot(2,1,2)
stairs(LOG_TP, LOG_U,style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
% xlim([0,max(LOG_STEP)]);

set(gcf,'position',[200,400,650,400]);
