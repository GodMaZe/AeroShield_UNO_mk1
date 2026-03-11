close all; clear;
clc;

addpath("../misc");

loadconfigs;
%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "pot2pwm";

COMPORT = "COM3";

[FILEPATH, FILEPATH_MAT, OUTPUT_NAMES] = prepareenv(FILENAME, "t,tp,y,u,pot,pct,dtp,dt,step,ref", DDIR);

%% Declare all the necessary variables
Tstop = 60;

Ts = 0.05;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

global LOGGER;

function plotdatarealtime()
    global LOGGER;
    persistent hy hr hu;
    % ----------------------------------
    % Plot the measured data in real time
    % ----------------------------------
    try
        if isempty(hy) || isempty(hr) || isempty(hu)
            f = figure("Name","Plot-RealTime");
            ax = axes(f);
            hold on;
            hy = stairs(ax, nan, nan);
            hr = stairs(ax, nan, nan);
            hu = stairs(ax, nan, nan);
            grid minor;
            title("Real-Time System Response");
            xlabel("t [s]");
            ylabel("$\varphi [^\circ]$", "Interpreter","latex");
            legend(ax, "y","ref", "u", 'Location', 'southeast');
            
        end
       
        nsteps = LOGGER.count;
        last_n_points = nsteps - 300;

        if last_n_points < 0
            last_n_points = 1;
        end

        mask = last_n_points:nsteps;
        t = LOGGER.get("tp", mask);

        set(hy, 'YData', LOGGER.get("y", mask), 'XData', t);
        set(hr, 'YData', LOGGER.get("ref", mask), 'XData', t);
        set(hu, 'YData', LOGGER.get("u", mask), 'XData', t);
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
    
    initconnection;
    
    LOGGER = DataLogger(dfile_handle, OUTPUT_NAMES, Ts);

    aerodata = AeroData;
    bytes = read(scon, aerodata.packetsize, "uint8");
    aerodata = aerodata.parse(bytes);

    disp(aerodata.tostring());

    init_plant_time = -1;
    plant_output = aerodata.output;
    plant_input = aerodata.control;
    plant_potentiometer = aerodata.potentiometer;
    plant_dt = aerodata.dt;
    plant_control_time = aerodata.controltime;

    time_start = datetime("now");
    time_curr = time_start;
    time_last = time_curr;

    plant_time_init = -1;
    plant_time = 0;
    step = 0;

    REF = 0;
    
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if step > 0 && time_delta < Ts
            continue;
        end

        write(scon, aerodata.potentiometer, "single");
        
        % Wait for the system to send a data message
        bytes = read(scon, aerodata.packetsize, "uint8");
        aerodata = aerodata.parse(bytes);

        if plant_time_init < 0
            plant_time_init = aerodata.time;
        end

        plant_time = aerodata.time - plant_time_init;
        plant_output = aerodata.output;
        plant_input = aerodata.control;
        plant_potentiometer = aerodata.potentiometer;
        plant_dt = aerodata.dt;
        plant_control_time = aerodata.controltime - plant_time_init;

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta, step, plant_control_time, REF];
        
        LOGGER = LOGGER.record(data, mod(step, 10) == 0);

        step = step + 1;
        time_last = time_curr;

        if plant_time >= Tstop || plant_output >= Ystop
            % configureCallback(scon, "off"); % Remove the callback from the serial port, before exiting the loop
            break;
        end
    end

catch er
    % Send a final command and close the serial port
    close_connection(scon, dfile_handle);
    rethrow(er);
end

%% close conns
close_connection(scon, dfile_handle);

%% Save the measurement
logsout = LOGGER.totable();
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================
figure(999);
style='-k';

subplot(2,1,1)
hold on;
plot(logsout.tp, logsout.ref, "--k","LineWidth", 1.5);
stairs(logsout.tp, logsout.y, 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('$\varphi(t) [\circ]$','Interpreter','latex');
grid minor;
grid on;
hold off;

subplot(2,1,2)
stairs(logsout.tp, logsout.u, style, 'LineWidth', 1.5)
xlabel('t [s]');
ylabel('u(t) [%PWM]');
grid minor;
grid on;
set(gcf,'position',[200,400,650,400]);
