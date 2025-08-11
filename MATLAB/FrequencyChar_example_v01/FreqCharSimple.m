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
    fprintf(datafileID, 't, tp, r, y, u, dt, i, f\n');
    
    % ----------------------------------
    % ----------------------------------
    
    
    % ----------------------------------
    % ----------------------------------
    
    % Define time parameters
    
    T_start = 0;
    
    T_sample = 0.010;      % [sec]
    
    % Define step parameters
    
    upb = [10, 20, 30, 40, 50, 60]; % The picked OPs, based on the linearity at and in close proximity of the point
    
    STEP_SIZE = 5; % [%] ; the magnitude of the sine wave
    STEP_REPS = 1; % [-] ; number of reps. per freq.
    STEP_TIME = 1; % [-] ; number of oscillations per freq.
    STEP_TIME_IN_RADS = 2*pi*STEP_TIME; % [rad] ; number of oscillations in radians
    
    STEP_STAB_TIME = 10; % [s] ; time for the system to stabilize at an operating point.
    STEP_STAB_DU = 0.02; % [%] ; the small increment between OPs to get there with the least amount of unecessary oscillations

    BET_STAB_TIME = 5; % [s] ; time for stabilizing the pendulum when increasing the frequency
    
    nops = length(upb);
    
    freq_init = 1; % [Hz]
    freq_end  = 3.5; % [Hz]
    freq_step = 0.1; % [Hz]
    
    
    freqs = freq_init:freq_step:freq_end;
    lfreqs = length(freqs);
    
    % Define STOP TIME
    
    T_stop = (ceil(sum(STEP_TIME_IN_RADS./freqs)/10)*10 + lfreqs * (BET_STAB_TIME - 1) + STEP_STAB_TIME) * nops; % [s]
    
    % ----------------------------------
    % ----------------------------------
    
    % PARALLEL COMPUTING
    
    function updateInfo(datafileID, dt, Ts, x)
        if ((dt/1000) > (Ts*1.05))
            fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f --\n', x);
        else
            fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', x);
        end
        fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', x);
    end
    
    
    DataInformer = parallel.pool.DataQueue;
    DataInformer.afterEach(@(x) updateInfo(datafileID, x(end - 2), T_sample, x));
    
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
    tmp_printlist = [0, plant_time, plant_potentiometer, plant_output, plant_input, T_sample * 1000, plant_current, 0];
    send(DataInformer, tmp_printlist);
    
    
    % ----------------------------------
    % ----------------------------------
try   
    
    % ----------------------------------
    % MEASURE THE DYNAMIC CHARACTERISTIC OF THE SYSTEM
    % ----------------------------------
    
    u_old = 0;
    
    for u=upb
        
        STEP_PROCESS_COUNTER = 1;
        u_send = u_old;
    
        START_STAB_TIME = true;
        START_BET_STAB_TIME = false;
        loop_init = false;
        
        % Get the initial time
        time_start = datetime('now');
        time_curr = time_start;
        time_tick = time_start;
        time_sample = time_start;
        time_stab = time_start;
        freq_current = 0;
        sin_val = 0;
        lsin_val = sin_val; % The last input value for the sine function
    
        u_step_counter = 1;
    
        % Main loop
        while true

            % Get current time
            if(~loop_init)
                loop_init = true;
            else
                time_curr = datetime('now');
            end

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
            if (~START_STAB_TIME && (sin_val) >= STEP_TIME_IN_RADS)
                time_sample = time_curr;
                lsin_val = sin_val;
                % The whole process order has finished, thus go back to start
                % for repetitions if enabled
                if STEP_PROCESS_COUNTER >= lfreqs
                    STEP_PROCESS_COUNTER = 1;
                    u_step_counter = u_step_counter + 1;
                    time_start = time_curr; % Reset the timer for each rep.
                else
                    STEP_PROCESS_COUNTER = STEP_PROCESS_COUNTER + 1;
                end


    
               
                if u_step_counter > STEP_REPS
                    % This last u-value sent to the system may be used while
                    % processing the measurement for a fixed point in the data.
                    u_send = u;
                    freq_current = 0;
                    START_STAB_TIME = true;
                end
    
            end
            
            % Check if it's time to send a new command
            if (time_delta >= T_sample * 1000)
                time_tick = time_curr;
                
                % Calculate total time elapsed
                time_elapsed = seconds(time_curr - time_start);
                
                % fprintf("%d %d %d %d\n", s2byte(u_send));
                if(~START_STAB_TIME && u_step_counter <= STEP_REPS)
                    freq_current = freqs(STEP_PROCESS_COUNTER);
                    sin_val = 2*pi*freq_current * time_elapsed; 
                    u_send = u + sin(sin_val) * STEP_SIZE;
                end

                % Send control input to the serial port
                % write(serPort, u_send, 'single');
                write(serPort, 0, 'single');
                
                % Read and parse the received data
                serLineList = str2num(readline(serPort)); %#ok<ST2NM>
                
                % Extract values from the received data
                plant_time = serLineList(1) - plant_time_init;
                plant_potentiometer = serLineList(2);
                plant_output = serLineList(3);
                plant_input = serLineList(4);
                plant_current = serLineList(5);
                
                % Display the received data
                tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, u_send, time_delta, plant_current, freq_current];
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
    freq_char = readtable(datafileName + ".csv", "VariableNamingRule","preserve","Delimiter",",");
    
    save(datafileName, "upb", "STEP_STAB_TIME", "STEP_SIZE", "T_sample", "T_start", "STEP_TIME", "T_stop", "STEP_REPS", "nops", "u", "freq_char", "freqs", "lfreqs", "STEP_STAB_DU","STEP_PROCESS_COUNTER");

catch er
    fprintf(2, 'Error while measuring the freq. characteristic of the system. Shutting down the connection...\n');
    write(serPort, 0.0, 'single');
    clear serPort;
    fclose(datafileID);
    rethrow(er);
end