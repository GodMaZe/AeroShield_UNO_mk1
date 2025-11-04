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
OUTPUT_NAMES = ["t", "tp", "y", "u", "pot", "dtp", "dt", "step", "pct"];

%% Declare all the necessary variables
Tstop = 60;

Ts = 0.05;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

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

    init_plant_time = sline(1);
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


    P = 0.1;
    I = 0.6;
    D = 0.025;

    e = 0;
    eint = 0;
    elast = 0;

    REF = 60;
    
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);
        time_last = time_curr;

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
        
        e = REF - plant_output;
        eint = eint + e;
        % eint = max(0, min(eint/I/plant_dt, 100/I/plant_dt));
        de = (e - elast);
        elast = e;
        
        uP = P * e;
        uI = I * eint * plant_dt;
        uD = D * de/plant_dt;

        u = uP + max(0, min(uI, 100)) + uD;

        u = max(0, min(100, u));

        write(scon, u, "single");
        
        % Wait for the system to send a data message
        sline = str2num(readline(scon));

        plant_time = sline(1) - plant_time_init;
        plant_output = sline(2);
        plant_input = sline(3);
        plant_potentiometer = sline(4);
        plant_dt = sline(5);
        plant_control_time = sline(6) - plant_time_init;

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta, plant_control_time];
        writenum2file(dfile_handle, data, mod(step, 10)==1, Ts, time_delta);

        LOG_T = [LOG_T, time_elapsed];
        LOG_TP = [LOG_TP, plant_time];
        LOG_CTRL_T = [LOG_CTRL_T, plant_control_time];
        LOG_Y = [LOG_Y, plant_output];
        LOG_U = [LOG_U, plant_input];
        LOG_POT = [LOG_POT, plant_potentiometer];
        LOG_DTP = [LOG_DTP, plant_dt];
        LOG_DT = [LOG_DT, time_delta];
        LOG_STEP = [LOG_STEP, step];
        step = step + 1;

        if plant_time >= Tstop || plant_output >= Ystop
            configureCallback(scon, "off"); % Remove the callback from the serial port, before exiting the loop
            break;
        end
    end

catch er
    % Send a final command and close the serial port
    if exist("scon", "var")
        write(scon, 0.0, 'single');
        scon.flush("input");
        clear scon;
    end
    if exist("dfile_handle", "var")
        fclose(dfile_handle);
        clear dfile_handle;
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

%% Save the measurement
logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_DTP, LOG_DT, LOG_STEP, LOG_CTRL_T, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================
figure(1);
style='-k';

subplot(2,1,1)
% stairs(LOG_TP,LOG_Y,style,'LineWidth',1.5)
scatter(LOG_TP, LOG_Y, '.k');
xlabel('k'); ylabel('\phi(k)'); grid on
% xlim([0,max(LOG_STEP)]);

subplot(2,1,2)
stairs(LOG_TP,LOG_U,style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
% xlim([0,max(LOG_STEP)]);

set(gcf,'position',[200,400,650,400]);
