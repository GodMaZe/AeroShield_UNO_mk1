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

FILENAME = "dataFile_comms";

function fullpath = getfilename(dirpath, filename, datestr, ext)
    if nargin < 3
        error("At least the first 3 parameters need to be provided.");
    end
    if nargin == 3
        ext = "csv";
    end

    fullpath = "./" + dirpath + "/" + filename + "_" + datestr + "." + ext;
end

FILEPATH = getfilename(DDIR, FILENAME, DateString);
FILEPATH_MAT = getfilename(DDIR, FILENAME, DateString, 'mat');

if(exist("datafileID", "var"))
    fclose(datafileID);
    clear datafileID;
end

datafileID = fopen(FILEPATH,'w');
fprintf(datafileID, 't, tp, r, y, u, dtp, dt\n');

% ----------------------------------
% ----------------------------------


% ----------------------------------
% ----------------------------------

% Define time parameters

T_start = 0;

T_sample = 3;      % [s] <1, 255>

% Define STOP TIME

T_stop = 10.0;     % [sec]



% Define control parameters

U_MAX = 100.0;
U_MIN = 0.0;
Y_SAFETY = 190.0;

% Define PID params.
P = 1.2475;
I = 0.2235;
D = 0.20;

R_WANTED = 170;


% ----------------------------------
% ----------------------------------

% PARALLEL COMPUTING

function updateInfo(datafileID, dt, Ts, x)
    if ((dt) > (Ts*1.05))
        fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f  --\n', x);
    else
        fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', x);
    end
    fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', x);
end


DataInformer = parallel.pool.DataQueue;
DataInformer.afterEach(@(x) updateInfo(datafileID, x(end), T_sample, x));

% ----------------------------------
% ----------------------------------

% Define serial port parameters and open
if(exist("serPort", "var"))
    clear serPort;
end

serPort = serialport('COM3', 115200, 'Timeout', 5);


serLine = readline(serPort);
while(~contains(serLine, "config"))
    disp(serLine);
    serLine = readline(serPort);
end

disp(serLine);
write(serPort, cast(T_sample, "uint8"), "uint8");

% Read the first line from the serial port (MCU starting)
while(~contains(serLine, "MCU"))
    disp(serLine);
    serLine = readline(serPort);
end

disp(serLine);
write(serPort, 0.0, 'single'); % Necessary to send this command for stable sampling period

while(contains(serLine, "---"))
    disp(serLine);
    serLine = readline(serPort);
end

% Read and parse the calibration data
serLineList = str2num(serLine); %#ok<ST2NM>
% 
% % Extract values from the received data
plant_time_init = serLineList(1);
% plant_potentiometer_init = serLineList(2);
% plant_output_init = serLineList(3);
% plant_input_init = serLineList(4);

plant_time = serLineList(1) - plant_time_init;
plant_input = serLineList(2);
plant_output = serLineList(3);
plant_potentiometer = serLineList(4);
plant_dt = serLineList(5);

% Display the received data
tmp_printlist = [0, plant_time, plant_potentiometer, plant_output, plant_input, plant_dt, T_sample];
send(DataInformer, tmp_printlist);


% Set initial control input value
e_old = 0;
e_int_old = 0;
u = 0;
u_send = u;


% Get the initial time
time_start = datetime('now');
time_tick = time_start;

% ----------------------------------
% ----------------------------------

function readSerialData(src, ~) 
    data = readline(src);
    src.UserData = data;
end

configureCallback(serPort, "terminator", @readSerialData);

% Main loop
while true
    waitfor(serPort, "UserData");

    % Get current time
    time_curr = datetime('now');

    % Calculate time elapsed since last iteration
    time_delta = milliseconds(time_curr - time_tick);

    % Read and parse the received data
    serLineList = str2num(serPort.UserData); %#ok<ST2NM>

    time_tick = time_curr;

    % Calculate total time elapsed
    time_elapsed = seconds(time_curr - time_start);

    % Extract values from the received data
    plant_time = serLineList(1) - plant_time_init;
    plant_input = serLineList(2);
    plant_output = serLineList(3);
    plant_potentiometer = R_WANTED; %*sin(2*pi*20*plant_time/1e6) + R_WANTED; % serLineList(4);
    plant_dt = serLineList(5);

    % Display the received data
    tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, plant_input, plant_dt, time_delta];
    send(DataInformer, tmp_printlist);


    e = plant_potentiometer - plant_output;

    e_der = (e - e_old) / (time_delta/1000);

    e_int = e_int_old + (e * (time_delta/1000));

    e_old = e;
    e_int_old = e_int;


    u = P * e  +  I * e_int + D * e_der;

    % u=plant_potentiometer;

    u_send = u;

    if u_send > U_MAX
        u_send = U_MAX;
    elseif u_send < U_MIN
        u_send = U_MIN;
    end

    % Send control input to the serial port
    write(serPort, u_send, "single");

    % ----------------------------------
    % ----------------------------------

    % Check if the simulation should stop
    if time_elapsed >= T_stop || plant_output >= Y_SAFETY
        configureCallback(serPort, "off"); % Remove the callback from the serial port, before exiting the loop
        break;
    end
end


% Send a final command and close the serial port
write(serPort, 0.0, 'single');
clear serPort;
fclose(datafileID);
clear datafileID;


logsout = readtable(FILEPATH, "VariableNamingRule","preserve","Delimiter",",");

save(FILEPATH_MAT, "U_MAX", "U_MIN", "Y_SAFETY", "T_sample", "T_start", "T_stop", "u", "P", "I", "D", "logsout");

t = logsout.t;
y = logsout.y;
u = logsout.u;
r = logsout.r;
e = r - y;
dt = logsout.dtp;


figure(111);
plot(t, y, '-k', 'LineWidth', 1.5);
hold on;
plot(t, r, '-r', 'LineWidth', 1.5);
title('Control Response');
subtitle("P = " + num2str(P) + ", I = " + num2str(I) + ", D = " + num2str(D));
legend('y(t)', 'ref(t)', "Location", "best");
xlabel('t [s]');
ylabel('y [deg]');
grid on;
hold off;