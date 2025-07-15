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

% Plot

% plot_window = 10;
% plot_idx_num = floor(plot_window/T_sample);
% 
% plot_t = nan(plot_idx_num,1);
% plot_sig_1 = nan(plot_idx_num,1);
% plot_sig_2 = nan(plot_idx_num,1);
% plot_sig_3 = nan(plot_idx_num,1);
% 
% figure(666);
% clf;

% Data save

DateString = convertCharsToStrings(datestr(datetime('now'), "yyyy_mm_dd_HH_MM_ss"));

datafileID = fopen("./" + DDIR + "/" + "dataFile_" + DateString + ".csv"','w');

% ----------------------------------
% ----------------------------------


% ----------------------------------
% ----------------------------------

% Define time parameters

T_start = 0;

T_sample =    0.02;      % [sec]


% step time
T_step_time = 5.0;         % [sec] 20.00

% Define step parameters

STEP_INIT = 0;

STEP_SIZE = 1;

STEP_MAX = 82.0;



% Define STOP TIME

T_stop = T_step_time * (STEP_MAX/STEP_SIZE + 1);     % [sec]

% ----------------------------------
% ----------------------------------

% PARALLEL COMPUTING

function updateInfo(datafileID, dt, Ts, x)
    if ((dt/1000) > (Ts*1.05))
        fprintf(2,'%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', x);
    else
        fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', x);
    end
    fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', x);
end


DataInformer = parallel.pool.DataQueue;
DataInformer.afterEach(@(x) updateInfo(datafileID, x(end), T_sample, x));

% ----------------------------------
% ----------------------------------

% Define serial port parameters and open
serPort = serialport('COM3', 115200, 'Timeout', 5);

% write(serPort, 0.0, 'single');

% Read the first line from the serial port
serLine = readline(serPort);
disp(serLine);

write(serPort, 0.0, 'single'); % Necessary to send this command for stable sampling period

% % Read and parse the calibration data
serLineList = str2num(readline(serPort)); %#ok<ST2NM>
% 
% % Extract values from the received data
plant_time_init = serLineList(1);
% plant_potentiometer_init = serLineList(2);
% plant_output_init = serLineList(3);
% plant_input_init = serLineList(4);

plant_time = serLineList(1) - plant_time_init;
plant_potentiometer = serLineList(2);
plant_output = serLineList(3);
plant_input = serLineList(4);

% plant_potentiometer = 100 * plant_potentiometer/1023;
% plant_output = 100 * (plant_output - 2727)/(2176);

% Display the received data
tmp_printlist = [0, plant_time, plant_potentiometer, plant_output, plant_input, T_sample * 1000];
fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', tmp_printlist);
fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', tmp_printlist);
% updateInfo(datafileID, T_sample*1000, T_sample, tmp_printlist);
send(DataInformer, tmp_printlist);

% ----------------------------------
% ----------------------------------



% ----------------------------------
% ----------------------------------

% Set initial control input value
u_send = 0;



e_old = 0;
e_int_old = 0;
u = STEP_INIT;
u_send = u;


% Get the initial time
time_start = datetime('now');
time_tick = time_start;
time_sample = time_start;

% ----------------------------------
% ----------------------------------

% Main loop
while true
    % Get current time
    time_curr = datetime('now');
    
    % Calculate time elapsed since last iteration
    time_delta = milliseconds(time_curr - time_tick);

    time_delta_sample = milliseconds(time_curr - time_sample);

    if time_delta_sample >= T_step_time * 1000
        time_sample = time_curr;
        u = u + STEP_SIZE;
        u_send = u;
        
        if u_send > STEP_MAX
            u_send = STEP_MAX;
        elseif u_send < 0
            u_send = 0.0;
        end        

    end


    
    % Check if it's time to send a new command
    if (time_delta >= T_sample * 1000)
        time_tick = time_curr;
        
        % Calculate total time elapsed
        time_elapsed = seconds(time_curr - time_start);
        
        % fprintf("%d %d %d %d\n", s2byte(u_send));

        % Send control input to the serial port
        % write(serPort, s2byte(u_send), 'uint8');
        write(serPort, u_send, 'single');
        
        % Read and parse the received data
        serLineList = str2num(readline(serPort)); %#ok<ST2NM>
        
        % Extract values from the received data
        plant_time = serLineList(1) - plant_time_init;
        plant_potentiometer = serLineList(2);
        plant_output = serLineList(3);
        plant_input = serLineList(4);

        % plant_potentiometer = 100 * plant_potentiometer/1023;
        % plant_output = 100 * (plant_output - 2727)/(2176);
        
        % Display the received data
        tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, plant_input, time_delta];
        % if ((time_delta/1000) > (T_sample*1.05))
        %     fprintf(2,'%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', tmp_printlist);
        % else
        %     fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', tmp_printlist);
        % end

        % fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', tmp_printlist);

        send(DataInformer, tmp_printlist);


        % ----------------------------------
        % ----------------------------------
        
        % plot_t = circshift(plot_t, -1);
        % plot_t(end) = time_elapsed;
        % 
        % plot_sig_1 = circshift(plot_sig_1, -1);
        % plot_sig_1(end) = plant_output;
        % 
        % plot_sig_2 = circshift(plot_sig_2, -1);
        % plot_sig_2(end) = plant_potentiometer;
        % 
        % plot_sig_3 = circshift(plot_sig_3, -1);
        % plot_sig_3(end) = plant_input;
        % 
        % plot(plot_t, plot_sig_3,'.b', plot_t, plot_sig_2,'.r', plot_t, plot_sig_1,'.k' )
        % xlim([min(plot_t), max(plot_t)+T_sample])
        % ylim([-10, 100]);
        % grid on;
        % legend("u","ref","y");
        % 
        % drawnow nocallbacks;
        
        % Check if the simulation should stop
        if time_elapsed >= T_stop || u > STEP_MAX
            break;
        end
    end
end





% Send a final command and close the serial port
% write(serPort, s2byte(0), 'uint8');
write(serPort, 0.0, 'single');
clear serPort;

fclose(datafileID);

save("./" + DDIR + "/" + "dataFile_" + DateString, "STEP_INIT", "STEP_MAX", "STEP_SIZE", "T_sample", "T_start", "T_step_time", "T_stop", "u");

