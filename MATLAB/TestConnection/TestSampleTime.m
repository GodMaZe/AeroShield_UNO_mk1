close all; clear;
clc;

addpath("./misc");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "dataFile";

if ~exist(DDIR, "dir")
    fprintf("Creating the default data repository folder, for saving the measurements...\n");
    mkdir(DDIR);
end

DateString = convertCharsToStrings(datestr(datetime('now'), "yyyy_mm_dd_HH_MM_ss"));

% File paths for the csv and mat files.
FILEPATH = getfilename(DDIR, FILENAME, DateString);
FILEPATH_MAT = getfilename(DDIR, FILENAME, DateString, 'mat');

% The names of the parameters to write into the file
OUTPUT_NAMES = ["t", "tp", "y", "u", "pot", "dtp", "dt", "step", "pct", "ref"];

%% Declare all the necessary variables
Tstop = 20;

Ts = 0.02;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

global LOG_T LOG_TP LOG_POT LOG_Y LOG_U LOG_DTP LOG_DT LOG_STEP LOG_CTRL_T LOG_REF;

% Logging vectors
LOG_T = [];
LOG_TP = [];
LOG_POT = [];
LOG_Y = [];
LOG_U = [];
LOG_DTP = [];
LOG_DT = [];
LOG_STEP = [];
LOG_CTRL_T = [];
LOG_REF = [];

function plotdatarealtime()
    global LOG_T LOG_Y LOG_U LOG_REF;
    persistent hy hr hu;
    % ----------------------------------
    % Plot the measured data in real time
    % ----------------------------------
    try
        if isempty(hy) || isempty(hr) || isempty(hu)
            f = figure("Name","Plot-RealTime","GraphicsSmoothing","on","Renderer","opengl","RendererMode","auto");
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
            legend(ax, "y","ref", "u", 'Location', 'southeast');
            
        end
       
        % plot(plot_t, plot_sig_3,'.b', plot_t, plot_sig_2,'.r', plot_t,
        % plot_sig_1,'.k')
        % print(timer_t(1));
        nsteps = numel(LOG_Y);
        last_n_points = nsteps - 300;

        if last_n_points < 0
            last_n_points = 1;
        end

        mask = last_n_points:nsteps;
        t = LOG_T(mask);

        set(hy, 'YData', LOG_Y(mask), 'XData', t);
        set(hr, 'YData', LOG_REF(mask), 'XData', t);
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

    scon = serialport("COM4", 115200, "Timeout", 5);
    
    sline = "";

    while(~contains(sline, "MCU"))
        % disp(sline);
        sline = readline(scon);
    end

    disp(sline);

    sline = str2num(readline(scon));
    disp(sline);

    init_plant_time = -1;
    plant_output = sline(2);
    plant_input = sline(3);
    plant_potentiometer = sline(4);
    plant_dt = sline(5);
    plant_control_time = sline(6);

    time_start = datetime("now");
    time_curr = time_start;
    time_last = time_curr;

    plant_time_init = -1;
    plant_time = 0;
    step = 0;

    % REF = plant_potentiometer;

    

    chrom = [0.1633    4.6267    0.159];
    % chrom = [0    2.2    0.1]; % Home
    % chrom = [0 2.75 0.05];
    % chrom = [0 3.09 0.06];
    chrom = [0.04 3.16 0.04];
    chrom = [0.09 6.57 0.08];
    chrom = [0.88 5.62 0.17]; % Fix PID in GA
    chrom = [0.39 3.85 0.11];

    P = chrom(1);
    I = chrom(2);
    D = chrom(3);

    % P = 0.01;
    % I = 1.5;
    % D = 0.0525;

    REF = 35;

    e = 0;
    eint = 0;
    elast = plant_output;

    U_PB = 30;
    u = 0;
    udt = 1;
    is_init = true;

    SYNC_TIME = 10;
    Tstop = Tstop + SYNC_TIME;
    nsteps = floor(Tstop/Ts);
    
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if time_delta < Ts
            continue;
        end

        % write(scon, 5*(sin(step/2/pi)) + 20, "single");

        % u = 10;
        % 
        % if time_elapsed >= 7.5
        %     u = 20;
        % elseif time_elapsed >= 5.0
        %     u = 10;
        % elseif time_elapsed >= 2.5
        %     u = 20;
        % else
        %     u = 10;
        % end
        % 
        % write(scon, u, "single");

        elapsed = time_elapsed - SYNC_TIME;

        if elapsed >= 15
            REF = 42.5;
        elseif elapsed >= 10
            REF = 40;
        elseif elapsed >= 5
            REF = 45;
        end

        % REF = plant_potentiometer;
        
        if is_init
            u = u + udt;
            u = max(0, min(u, U_PB));
            if u >= U_PB
                is_init = false;
            end
        else
            u = U_PB;
        end
        
        if elapsed >= 0
            e = REF - plant_output;
            eint = max(-85, min(85, (eint + e * time_delta)));
            % eint = max(0, min(eint/I/plant_dt, 100/I/plant_dt));
            de = -(plant_output - elast)/time_delta;
            
            uP = P * e;
            uI = I * eint;
            uD = D * de;
            % fprintf("u: %8.3f | uP: %8.3f | uI: %8.3f | uD: %8.3f\n", u, uP, uI, uD);
            u = u + uP + uI + uD;
            
            u = max(0, min(100, u));
        end

        elast = plant_output;

        write(scon, u, "single");
        
        % Wait for the system to send a data message
        sline = str2num(readline(scon));

        if plant_time_init < 0
            plant_time_init = sline(1);
        end

        plant_time = sline(1) - plant_time_init;
        plant_output = sline(2);
        plant_input = sline(3);
        plant_potentiometer = sline(4);
        plant_dt = sline(5);
        plant_control_time = sline(6) - plant_time_init;

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta, plant_control_time, REF];
        writenum2file(dfile_handle, data, mod(step, 10)==0, Ts, time_delta);

        LOG_T = [LOG_T, time_elapsed];
        LOG_TP = [LOG_TP, plant_time];
        LOG_CTRL_T = [LOG_CTRL_T, plant_control_time];
        LOG_Y = [LOG_Y, plant_output];
        LOG_U = [LOG_U, plant_input];
        LOG_POT = [LOG_POT, plant_potentiometer];
        LOG_DTP = [LOG_DTP, plant_dt];
        LOG_DT = [LOG_DT, time_delta];
        LOG_STEP = [LOG_STEP, step];
        LOG_REF = [LOG_REF, REF];

        step = step + 1;
        time_last = time_curr;

        if plant_time >= Tstop || plant_output >= Ystop
            % configureCallback(scon, "off"); % Remove the callback from the serial port, before exiting the loop
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
logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_DTP, LOG_DT, LOG_STEP, LOG_CTRL_T, LOG_REF, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================

figure(1); clf;
hold on;
stairs(LOG_TP, LOG_Y);
stairs(LOG_TP, LOG_REF);
stairs(LOG_TP, LOG_U);
title("Real-Time System Response");
xlabel("t [s]");
ylabel("$\varphi [^\circ]$", "Interpreter","latex");
legend("y","ref", "u", 'Location', 'southeast');
grid minor;
hold off;


figure(999);
style='-k';

subplot(2,1,1)
hold on;
plot(LOG_TP, LOG_REF,"--k","LineWidth",1.5);
stairs(LOG_TP,LOG_Y,'LineWidth',1.5);
% scatter(LOG_TP, LOG_Y, '.k');
xlabel('k'); ylabel('\phi(k)'); grid on
% xlim([0,max(LOG_STEP)]);
hold of;

subplot(2,1,2)
stairs(LOG_TP,LOG_U,style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
% xlim([0,max(LOG_STEP)]);

set(gcf,'position',[200,400,650,400]);
