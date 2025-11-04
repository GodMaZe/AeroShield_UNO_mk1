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
OUTPUT_NAMES = ["t", "tp", "y", "u", "pot", "dtp", "dt", "step"];

%% Declare all the necessary variables
Tstart = 0;
Tstop = 10;
Ts = 0.05;
nsteps = floor((Tstop - Tstart)/Ts);

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
        serPort.flush("input");
        clear scon;
    end

    scon = serialport("COM3", 9600, "Timeout", 5);
    sline = readline(scon);
    disp(sline);

    time_start = datetime("now");
    time_curr = time_start;
    time_last = time_curr;

    plant_time_init = -1;
    
    for step=1:nsteps
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);
        time_last = time_curr;
        
        % Wait for the system to send a data message
        sline = str2num(readline(scon));

        % Extract values from the received data
        if plant_time_init < 0
            plant_time_init = sline(1);
        end

        plant_time = sline(1) - plant_time_init;
        plant_output = sline(2);
        plant_input = sline(3);
        plant_potentiometer = sline(4);
        plant_dt = sline(5);

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta];
        writenum2file(dfile_handle, data, 1, Ts, time_delta);

        LOG_T = [LOG_T, time_elapsed];
        LOG_TP = [LOG_TP, plant_time];
        LOG_Y = [LOG_Y, plant_output];
        LOG_U = [LOG_U, plant_input];
        LOG_POT = [LOG_POT, plant_potentiometer];
        LOG_DTP = [LOG_DTP, plant_dt];
        LOG_DT = [LOG_DT, time_delta];
        LOG_STEP = [LOG_STEP, step];

        write(scon, 10.0, "single");

        if time_elapsed >= Tstop || plant_output >= Ystop
            configureCallback(scon, "off"); % Remove the callback from the serial port, before exiting the loop
            break;
        end
    end

catch er
    % Send a final command and close the serial port
    write(scon, 0.0, 'single');
    scon.flush("input");
    clear scon;
    fclose(dfile_handle);
    clear dfile_handle;
    rethrow(er);
end

write(scon, 0.0, 'single');
scon.flush("input");
clear scon;
fclose(dfile_handle);
clear dfile_handle;

logsout = table(LOG_T, LOG_TP, LOG_Y, LOG_U, LOG_POT, LOG_DTP, LOG_DT, LOG_STEP, 'VariableNames', OUTPUT_NAMES);
save(FILEPATH_MAT, "Tstart", "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================
figure
style='-k';

subplot(2,1,1)
stairs(LOG_STEP,LOG_Y,style,'LineWidth',1.5)
xlabel('k'); ylabel('\phi(k)'); grid on
xlim([0,max(LOG_STEP)]);

subplot(2,1,2)
stairs(LOG_STEP,LOG_U,style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
xlim([0,max(LOG_STEP)]);

set(gcf,'position',[0,200,650,400]);
