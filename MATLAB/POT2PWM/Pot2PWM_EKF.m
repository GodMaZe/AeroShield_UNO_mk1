close all; clear;
clc;

addpath("../misc");
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");

loadconfigs;

load("../misc/models/ipendulum_model");
pendulum = sys;
pendulum.Ku = pendulum.Ku/0.72;

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "pot2pwm";

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
Tstop = 30;

Ts = 0.05;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

global LOG_T LOG_TP LOG_POT LOG_Y LOG_U LOG_DTP LOG_DT LOG_STEP LOG_CTRL_T LOG_REF LOG_Y_HAT LOG_X_HAT LOG_P_HAT;

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
LOG_Y_HAT = [];
LOG_X_HAT = [];
LOG_P_HAT = [];

function plotdatarealtime()
    global LOG_T LOG_Y LOG_U LOG_Y_HAT;
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
            legend(ax, "y","yhat", "u", 'Location', 'southeast');
            
        end
       
        nsteps = numel(LOG_Y);
        last_n_points = nsteps - 300;

        if last_n_points < 0
            last_n_points = 1;
        end

        mask = last_n_points:nsteps;
        t = LOG_T(mask);

        set(hy, 'YData', LOG_Y(mask), 'XData', t);
        set(hr, 'YData', rad2deg(LOG_Y_HAT(mask)), 'XData', t);
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

    scon = serialport("COM3", CF_BAUDRATE, "Timeout", CF_TIMEOUT);
    
    sline = "";

    while(~contains(sline, "MCU"))
        % disp(sline);
        sline = readline(scon);
    end

    disp(sline);

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
    u = aerodata.control;

    %% Initialize Extended Kalman Filter
    w_disturbance = true;

    [pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, w_disturbance);

    n = pendulum.n;
    r = pendulum.r;
    m = pendulum.m;

    x0 = zeros(n, 1);
    x0(1) = deg2rad(aerodata.output);

    R = (0.015)^2; % Measurement noise (from datasheet)
    if w_disturbance
        Q = diag([(0.001)^2 (0.001*Ts)^2 1^2]);
    else
        Q = diag([(0.001)^2 (0.001*Ts)^2]);
    end

    P = diag(ones(size(x0))*var(x0));

    ekf = ExtendedKalmanFilter(f, h, x0, 1, 'R', R, 'Q', Q, 'P', P, 'epstol', Ts);
    
    %% Insert null entries into the Kalman logs
    [ekf, y_hat] = ekf.step(plant_time, u);
    % LOG_Y_HAT = [LOG_Y_HAT, y_hat];
    % LOG_X_HAT = [LOG_X_HAT, ekf.get_xhat()];
    % LOG_P_HAT = [LOG_P_HAT, ekf.P];
    

    %% Start the loop
    

    while plant_time <= Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if step > 0 && time_delta < Ts
            continue;
        end

        u = aerodata.potentiometer;

        write(scon, u, "single");
        
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

        [ekf, y_hat] = ekf.step(plant_time, u, deg2rad(aerodata.output));

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta, step, plant_control_time, REF];
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
        LOG_Y_HAT = [LOG_Y_HAT, y_hat];
        LOG_X_HAT = [LOG_X_HAT, ekf.get_xhat()];
        LOG_P_HAT = [LOG_P_HAT, ekf.P];


        step = step + 1;
        time_last = time_curr;

        if plant_time >= Tstop || plant_output >= Ystop
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
logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_DTP, LOG_DT, LOG_STEP, LOG_CTRL_T, LOG_REF, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================
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

%% Plot the Kalman parameters
addpath("../misc/plotting");

plt = Data2Plot(LOG_TP, rad2deg(LOG_X_HAT),[],"stairs","s","^\circ","EKF Estimate");
plt.plotx();

plt = Data2Plot(LOG_TP, LOG_Y, rad2deg(LOG_Y_HAT),"stairs","s","^\circ","Output vs EKF Estimate","Estimation comparsion");
plt.plotoutnerror(123, 0, "./figures/out_vs_ekf");