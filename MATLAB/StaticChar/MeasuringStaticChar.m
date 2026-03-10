close all; clear;
clc;

addpath("../misc");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "mer1_statChar_du5";
% mer# - measurement count per fixed configuration
% statChar - static characteristic
% du## - step size, where du04 is step size of 0.4, du4 is step size of
% 4.0, and du40 is step size of 40.0, therefore du## step size is
% determined based on whether a it contains a preceding zero or
% postceding.

COMPORT = "COM3"; % Serial port of the device

[FILEPATH, FILEPATH_MAT, OUTPUT_NAMES] = prepareenv(FILENAME, "t,tp,y,u,pot,pct,dtp,dt,step,ref", DDIR);

%% Declare all the necessary variables
U_STEP_SIZE = 5; % max resolution per step based on the following calc: 100/255 = 0.3922, where 100 is max %PWM and 255 is the 8bit value for the PWM generator
STEPS = 0:U_STEP_SIZE:100;
SYNC_TIME = 20; % Time for the system to stabilize in the OP

Tstop = numel(STEPS) * SYNC_TIME;

Ts = 0.05;

Tstop = Tstop + SYNC_TIME;
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

try

    timerplotrealtime = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotdatarealtime());
    start(timerplotrealtime);

    % Will create variables: scon (serial connection), dfile_handle (data
    % file handle) and last but not least will initialize the serial
    % communication (waiting for the initial --- MCU started --- message)
    initconnection;
    
    aerodata = AeroData;
    bytes = read(scon, aerodata.packetsize, "uint8");

    %Init data if needed.
    aerodata = aerodata.parse(bytes);

    disp(aerodata.tostring());

    time_start = datetime("now");
    time_curr = time_start;
    time_last = time_curr;
    time_step = time_curr;

    plant_time_init = -1;
    plant_time = 0;
    

    REF = 0;

    step = 0;
    step_u = 1;
    u = 0;
    udt = U_STEP_SIZE/10;
    is_init = true;
    
    
    while aerodata.time < Tstop
        % --- Timing algorithm

        time_curr = datetime("now");
        
        if(step == 0)
            time_start = time_curr;
        end
        
        time_delta = seconds(time_curr - time_last);

        if step > 0 && time_delta < Ts
            continue;
        end

        % --- END Timing algorithm

        % --- Control algorithm

        time_step_delta = seconds(time_curr - time_step);

        if time_step_delta >= SYNC_TIME
            time_step = time_curr;
            step_u = step_u + 1;
            is_init = true;
        end

        if is_init
            u = u + udt;
            u = max(0, min(u, STEPS(step_u)));
            if u >= STEPS(step_u)
                is_init = false;
            end
        else
            u = STEPS(step_u);
        end

        write(scon, u, "single");

        % --- END of Control algorithm
        
        % --- Measuring algorithm

        % Wait for the system to send a data message
        bytes = read(scon, aerodata.packetsize, "uint8");
        aerodata = aerodata.parse(bytes, true, true, step==0);

        % Write the data into a file
        time_elapsed = seconds(time_curr - time_start);
        data = [time_elapsed, aerodata.toarray(), time_delta, step, REF];
        writenum2file(dfile_handle, data, mod(step, 10)==0, Ts, time_delta);

        LOG_T = [LOG_T, time_elapsed];
        LOG_TP = [LOG_TP, aerodata.time];
        LOG_CTRL_T = [LOG_CTRL_T, aerodata.controltime];
        LOG_Y = [LOG_Y, aerodata.output];
        LOG_U = [LOG_U, aerodata.control];
        LOG_POT = [LOG_POT, aerodata.potentiometer];
        LOG_DTP = [LOG_DTP, aerodata.dt];
        LOG_DT = [LOG_DT, time_delta];
        LOG_STEP = [LOG_STEP, step];
        LOG_REF = [LOG_REF, REF];

        time_last = time_curr;
        step = step + 1;

        if aerodata.time >= Tstop || aerodata.output >= Ystop || aerodata.potentiometer > 50
            break;
        end
        % --- END Measuring algorithm
    end

catch er
    % Send a final command and close the serial port
    close_connection(scon, dfile_handle);
    rethrow(er);
end

%% close conns
close_connection(scon, dfile_handle);

%% Save the measurement
logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_CTRL_T, LOG_DTP, LOG_DT, LOG_STEP, LOG_REF, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT);

%%
% ===========================
%   Plot Results
% ===========================
FIGDIR = "./fig";
if ~exist(FIGDIR, "dir")
    mkdir(FIGDIR);
    fprintf("Figure directory not found, creating one...\n");
end

figure(1); clf;
hold on;
stairs(LOG_TP, LOG_Y, 'LineWidth', 1.5);
stairs(LOG_TP, LOG_U, 'LineWidth', 1.5);
title("Real-Time System Response");
xlabel("t [s]");
ylabel("$\varphi \ [^\circ]$", "Interpreter","latex");
legend("y", "u", 'Location', 'southeast');
grid minor;
hold off;
set(gcf,'position',[200, 400, 650, 400]);
saveas(gcf, FIGDIR + "/sys_response.svg", "svg");

figure(999); clf;
style='-k';

subplot(2,1,1)
hold on;
stairs(LOG_TP,LOG_Y,'LineWidth', 1.5);
xlabel('t [s]');
ylabel("$\varphi \ [^\circ]$", "Interpreter", "latex");
grid minor;
grid on;
hold of;
title("System response");


subplot(2,1,2)
plot(LOG_U, LOG_Y,'.k','LineWidth', 1.5)
xlabel('u [%PWM]');
ylabel("$\varphi \ [^\circ]$", "Interpreter", "latex");
grid minor;
grid on;
title("U to Y response");
set(gcf,'position',[200, 400, 650, 400]);
saveas(gcf, FIGDIR + "/sys_response_subplot.svg", "svg");
