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

datafileID = fopen("./" + DDIR + "/" + "dataFile_" + DateString + ".csv",'w');
fprintf(datafileID, 't, tp, r, y, u, dt\n');

% ----------------------------------
% ----------------------------------


% ----------------------------------
% ----------------------------------

% Define time parameters

T_start = 0;

T_sample =    0.02;      % [sec]

% Define STOP TIME

T_stop = 60.0;     % [sec]



% Define control parameters

U_MAX = 100.0;
U_MIN = 0.0;
Y_SAFETY = 150.0;

% Define PID params.

% okolie r = <56.6-73>
P = 0.0125;
I = 1.55;
D = 0.15125;

Ys = [0,             10,        20,      50,      70,       90,      110]';
Ps = [0.0125,    0.0105,    0.0165,  0.0125,  0.0105,   0.1005,   0.0055]';
Is = [0.85,       0.985,      1.05,    1.25,   1.175,    1.015,    0.965]';
Ds = [0.05125,   0.0725,  0.085125, 0.15125, 0.25125, 0.355125, 0.300125]';

RegParams = nan(ceil(T_stop/T_sample), 3);
RegParams(1, :) = [0, 0, 0];


pidtable = table(Ys, Ps, Is, Ds, 'VariableNames', {'y', 'p', 'i', 'd'});



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
% fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', tmp_printlist);
% fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', tmp_printlist);
% updateInfo(datafileID, T_sample*1000, T_sample, tmp_printlist);
send(DataInformer, tmp_printlist);

% ----------------------------------
% ----------------------------------



% ----------------------------------
% ----------------------------------

% Set initial control input value
e_old = 0;
e_int_old = 0;
u = 0;
u_send = u;


% Get the initial time
time_start = datetime('now');
time_tick = time_start;
RegParamsCounter = 2;

% ----------------------------------
% ----------------------------------

% Main loop
while true
    % Get current time
    time_curr = datetime('now');
    
    % Calculate time elapsed since last iteration
    time_delta = milliseconds(time_curr - time_tick);
    
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
        
        % Display the received data
        tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, plant_input, time_delta];

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
        % drawnow nocallbacks    


        % ----------------------------------
        % ----------------------------------

        e = plant_potentiometer - plant_output;

        e_der = (e - e_old) / (time_delta/1000);

        e_int = e_int_old + (e * (time_delta/1000));

        e_old = e;
        e_int_old = e_int;

        % alpha = 1.15283;

        % Fe = alpha * e + e_der;
        % u = U_MAX * tanh(0.015*Fe) + I * e_int;

        Ri = find(pidtable.y <= plant_output);
        if isempty(Ri)
            Ri = 1;
        else
            Ri = Ri(end);
        end


        P = pidtable.p(Ri);
        I = pidtable.i(Ri);
        D = pidtable.d(Ri);

        RegParams(RegParamsCounter, :) = [P, I, D];
        RegParamsCounter = RegParamsCounter + 1;

        u = P * e  +  I * e_int + D * e_der;

        % e = 0 - plant_output;

        % Stop controller
        b0 = 1; % 0.1
        gamma = 0.725; % 0.725
        dk = (gamma*log(abs(e) + b0))^2;
        u = u + (dk*log10(e^2 + b0));
        % u=0;

        u_send = u;
        
        if u_send > U_MAX
            u_send = U_MAX;
        elseif u_send < U_MIN
            u_send = U_MIN;
        end  

        % ----------------------------------
        % ----------------------------------
        
        % Check if the simulation should stop
        if time_elapsed >= T_stop || plant_output >= Y_SAFETY 
            break;
        end
    end
end





% Send a final command and close the serial port
% write(serPort, s2byte(0), 'uint8');
write(serPort, 0.0, 'single');
clear serPort;
fclose(datafileID);

RegParams = rmmissing(RegParams);

logsout = readtable("./" + DDIR + "/" + "dataFile_" + DateString + ".csv", "VariableNamingRule","preserve","Delimiter",",");

save("./" + DDIR + "/" + "dataFile_" + DateString, "U_MAX", "U_MIN", "Y_SAFETY", "T_sample", "T_start", "T_stop", "u", "P", "I", "D", "logsout", "RegParams");

