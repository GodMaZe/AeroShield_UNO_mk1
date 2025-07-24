clear;
clc; close all;

format short;

% delete(gcp('nocreate'));
% parpool('Processes', 2);


DDIR = "dataRepo";
if ~exist(DDIR, "dir")
    fprintf("Creating the data directory...");
    mkdir(DDIR);
end

% ----------------------------------
% ----------------------------------

% Data save

DateString = convertCharsToStrings(datestr(datetime('now'), "yyyy_mm_dd_HH_MM_ss"));
datafileName = "./" + DDIR + "/" + "dataFile_" + DateString;
datafileID = fopen(datafileName + ".csv"','w');
fprintf(datafileID, 't, tp, r, y, u, dt, i\n');

% ----------------------------------
% ----------------------------------


% ----------------------------------
% ----------------------------------

% Define time parameters

T_start = 0;

T_sample = 0.010;      % [sec]

% Define step parameters

upb = [10, 20, 30, 40, 50, 60]; % The picked OPs, based on the linearity at and in close proximity of the point

STEP_SIZE = 5; % [%] ; the size of the step in a direction

STEP_REPS = 1; % Number of repetitions per an operating point ; STEP UP - BACK TO Upb - STEP DOWN - BACK TO Upb (1 rep)

STEP_TIME = 10; % [s] ; time per each step

STEP_STAB_TIME = 20; % [s] ; time for the system to stabilize at an operating point.
STEP_STAB_DU = 0.02; % [%] ; the small increment between OPs to get there with the least amount of unecessary oscillations

nops = length(upb);

STEP_PROCESS_ORDER = [1, 0, -1, 0]; % STEP_TIME of upb, STEP_TIME of upb + STEP_SIZE, STEP_TIME of upb, STEP_TIME of upb - STEP_SIZE

nprocessorder = length(STEP_PROCESS_ORDER);

% Define STOP TIME

T_stop = (STEP_TIME * (nprocessorder * STEP_REPS) + STEP_STAB_TIME) * nops; % [s]

% ----------------------------------
% ----------------------------------

% PARALLEL COMPUTING

function updateInfo(datafileID, dt, Ts, x)
    if ((dt/1000) > (Ts*1.05))
        fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f --\n', x);
    else
        fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', x);
    end
    fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', x);
end


DataInformer = parallel.pool.DataQueue;
DataInformer.afterEach(@(x) updateInfo(datafileID, x(end - 1), T_sample, x));

% ----------------------------------
% CONNECT TO THE DEVICE AND RECORD THE CALIBRATION DATA
% ----------------------------------

% Define serial port parameters and open
serPort = serialport('COM3', 115200, 'Timeout', 5);

% write(serPort, 0.0, 'single');

% Read the first line from the serial port
serLine = readline(serPort);
disp(serLine);

write(serPort, 0.0, 'single'); % Necessary to send this command for stable sampling period

% Read and parse the calibration data
serLineList = str2num(readline(serPort)); %#ok<ST2NM>
 
% Extract values from the received data
plant_time_init = serLineList(1);
% plant_potentiometer_init = serLineList(2);
% plant_output_init = serLineList(3);
% plant_input_init = serLineList(4);

plant_time = serLineList(1) - plant_time_init;
plant_potentiometer = serLineList(2);
plant_output = serLineList(3);
plant_input = serLineList(4);
plant_current = serLineList(5);

% Display the received data
tmp_printlist = [0, plant_time, plant_potentiometer, plant_output, plant_input, T_sample * 1000, plant_current];
send(DataInformer, tmp_printlist);


% ----------------------------------
% ----------------------------------


% ----------------------------------
% MEASURE THE DYNAMIC CHARACTERISTIC OF THE SYSTEM
% ----------------------------------

u_old = 0;

for u=upb
    
    STEP_PROCESS_COUNTER = 1;
    u_send = u_old;

    START_STAB_TIME = true;
    
    
    % Get the initial time
    time_start = datetime('now');
    time_tick = time_start;
    time_sample = time_start;
    time_stab = time_start;

    u_step_counter = 1;

    % Main loop
    while true
        % Get current time
        time_curr = datetime('now');
        
        % Calculate time elapsed since last iteration
        time_delta = milliseconds(time_curr - time_tick);
    
        time_stab_delta = milliseconds(time_curr - time_stab);

        if (START_STAB_TIME && time_delta >= T_sample * 1000)
            u_send = u_send + STEP_STAB_DU;
            if u_send >= u
                u_send = u;
            end
        end

        % Wait for the system to stabilize at the new OP
        if (START_STAB_TIME && time_stab_delta >= STEP_STAB_TIME * 1000)
            START_STAB_TIME = false;
            time_start = time_curr;
        end

        time_delta_sample = milliseconds(time_curr - time_sample);
    
        % Change the step in order of the step process order array
        if (~START_STAB_TIME && time_delta_sample >= STEP_TIME * 1000)
            time_sample = time_curr;

            % The whole process order has finished, thus go back to start
            % for repetitions if enabled
            if STEP_PROCESS_COUNTER > nprocessorder
                STEP_PROCESS_COUNTER = 1;
                u_step_counter = u_step_counter + 1;
                time_start = time_curr; % Reset the timer for each rep.
            end

            u_send = u + STEP_PROCESS_ORDER(STEP_PROCESS_COUNTER) * STEP_SIZE;

            STEP_PROCESS_COUNTER = STEP_PROCESS_COUNTER + 1;

            if u_step_counter > STEP_REPS
                % This last u-value sent to the system may be used while
                % processing the measurement for a fixed point in the data.
                u_send = u;
            end

        end
        
        % Check if it's time to send a new command
        if (time_delta >= T_sample * 1000)
            time_tick = time_curr;
            
            % Calculate total time elapsed
            time_elapsed = seconds(time_curr - time_start);
            
            % fprintf("%d %d %d %d\n", s2byte(u_send));
    
            % Send control input to the serial port
            write(serPort, u_send, 'single');
            
            % Read and parse the received data
            serLineList = str2num(readline(serPort)); %#ok<ST2NM>
            
            % Extract values from the received data
            plant_time = serLineList(1) - plant_time_init;
            plant_potentiometer = serLineList(2);
            plant_output = serLineList(3);
            plant_input = serLineList(4);
            plant_current = serLineList(5);
            
            % Display the received data
            tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, plant_input, time_delta, plant_current];
            send(DataInformer, tmp_printlist);
    
            
            % Check if the simulation should stop
            if time_elapsed >= T_stop || u_step_counter > STEP_REPS || plant_potentiometer > 90
                break;
            end
        end
    end


    u_old = u;

    if time_elapsed >= T_stop || plant_potentiometer > 90
        break;
    end

end


% ----------------------------------
% ----------------------------------


% Send a final command and close the serial port
write(serPort, 0.0, 'single');
clear serPort;
fclose(datafileID);


% Save the measurements
dyn_char = readtable(datafileName + ".csv", "VariableNamingRule","preserve","Delimiter",",");

save(datafileName, "upb", "STEP_STAB_TIME", "STEP_SIZE", "T_sample", "T_start", "STEP_TIME", "T_stop", "STEP_REPS", "nops", "u", "dyn_char", "STEP_PROCESS_ORDER", "nprocessorder", "STEP_STAB_DU");

