format short;

function [timer_t, timer_y, timer_u, timer_potentiometer, U_PB] = runArduinoPlot()
    
    % ----------------------------------
    % Create the data repository folder
    % ----------------------------------
    
    DDIR = "dataRepo";
    if ~exist(DDIR, "dir")
        fprintf("Creating the data directory...");
        mkdir(DDIR);
    end

    % ----------------------------------
    % ----------------------------------

    % ----------------------------------
    % Define simulation parameters
    % ----------------------------------
    
    T_start = 0;
    
    T_sample = 3;      % [ms]
    
    STEP_SIZE = 5; % [%PWM]
    
    U_PB = 20; % [%PWM] pracovny bod, OP - operating point
    
    T_step_time = 15; % [sec]
    
    STEP_SHAPE = [1, 0, -1, 0, 1, -1, 1, 0]; % step up, down, down, up, up, large down, large up, down
    
    nsteps = length(STEP_SHAPE);
    
    T_stabilize_time = 30; % [sec] Cas stabilizacie v PB
    
    % Define control parameters
    
    U_MAX = 100.0;
    U_MIN = 0.0;
    Y_SAFETY = 100.0;
    
    % Define STOP TIME
    
    T_stop = T_stabilize_time + T_step_time * nsteps;  % [sec]
    
    % Define U increments to get to the U_PB smoothly
    dU = ceil(U_PB/T_stabilize_time*10)*T_sample/1000;
    
    
    fprintf(2, "Simulation will be running for the next: %8.3f seconds, with %8.3f steps\n", T_stop, nsteps + 1);
    
    % ----------------------------------
    % ----------------------------------
    
    % ----------------------------------
    % Define global lists for real-time rendering
    % ----------------------------------
    
    timer_t = [];
    timer_y = [];
    timer_u = [];
    timer_potentiometer = [];

    % ----------------------------------
    % ----------------------------------
    
    % ----------------------------------
    % Define and Initialize threaded plot rendering
    % ----------------------------------
    
    function plotData()
        persistent hy hr hu;
        % ----------------------------------
        % Plot the measured data in real time
        % ----------------------------------
        try
            if isempty(hy) || isempty(hr) || isempty(hu)
                f = figure(9999); clf(f);
                ax = axes(f);
                hold on;
                hy = plot(ax, nan, nan, '.b');
                hr = plot(ax, nan, nan, '.r');
                hu = plot(ax, nan, nan, '.k');
                grid minor;
                title("Real-Time System Response");
                xlabel("t [s]");
                ylabel("$\varphi [^\circ]$", "Interpreter","latex");
                legend(ax, "y","ref", "yhat", 'Location', 'southeast');
                
            end
           
            % plot(plot_t, plot_sig_3,'.b', plot_t, plot_sig_2,'.r', plot_t,
            % plot_sig_1,'.k')
            % print(timer_t(1));

            set(hy, 'YData', timer_y, 'XData', timer_t);
            set(hr, 'YData', timer_potentiometer, 'XData', timer_t);
            set(hu, 'YData', timer_u, 'XData', timer_t);
            drawnow limitrate nocallbacks;
        catch err
           fprintf(2, "Plot thread: " + err.message + "\n");
        end
        % ----------------------------------
        % ----------------------------------
    end
    
    tPlot = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotData());
    start(tPlot);

    % ----------------------------------
    % ----------------------------------

    % ----------------------------------
    % Define and Initialize File Streams
    % ----------------------------------
    
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
    % Record measurement data
    % ----------------------------------
    
    function updateInfo(datafileID, dt, Ts, x)
        if ((dt) > (Ts*1.05))
            fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f  --\n', x);
        else
            fprintf('%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n', x);
        end
        fprintf(datafileID, '%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n', x);
        timer_t = [timer_t x(1)];
        timer_y = [timer_y x(4)];
        timer_u = [timer_u x(5)];
        timer_potentiometer = [timer_potentiometer x(3)];
    end
    
    doUpdate = @(x) updateInfo(datafileID, x(end), T_sample, x);
    
    % ----------------------------------
    % ----------------------------------
    
    % Define serial port parameters and open
    if(exist("serPort", "var"))
        serPort.flush("input");
        clear serPort;
    end
    
    serPort = serialport('COM3', 115200, 'Timeout', 5);
    
    serLine = readline(serPort);
    
    while(~contains(serLine, "config"))
        disp(serLine);
        serLine = readline(serPort);
    end
    
    fprintf("Sending now\n");
    write(serPort, cast(T_sample, "uint8"), "uint8");
    
    

    % Read the first line from the serial port (MCU starting)
    while(~contains(serLine, "start"))
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
    doUpdate(tmp_printlist);
    
    
    % Set initial control input value
    u = 0;
    u_send = u;
    istep = 1; % Holding the step index within the STEP_SHAPE matrix
    isstable = false;
    
    
    % Get the initial time
    time_start = datetime('now');
    time_tick = time_start;
    time_step = time_start;
    time_stabilize = time_start;
    
    
    
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

        if (~isstable && milliseconds(time_curr - time_stabilize)/1000 >= T_stabilize_time)
            isstable = true;
        end
    
        if (isstable && milliseconds(time_curr - time_step)/1000 >= T_step_time)
            time_step = time_curr;
            u = U_PB + STEP_SHAPE(istep) * STEP_SIZE;
            istep = istep + 1;
        end
    
        % Calculate total time elapsed
        time_elapsed = seconds(time_curr - time_start);
    
        % Extract values from the received data
        plant_time = serLineList(1) - plant_time_init;
        plant_input = serLineList(2);
        plant_output = serLineList(3);
        plant_potentiometer = serLineList(4); % R_WANTED; %*sin(2*pi*20*plant_time/1e6) + R_WANTED; % serLineList(4);
        plant_dt = serLineList(5);
    
        % Display the received data
        tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, plant_input, plant_dt, time_delta];
        doUpdate(tmp_printlist);
    
    
        if(~isstable && u < U_PB)
            u = min(u + dU, U_PB);
        end

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
    
    for tim=timerfindall
        stop(tim);
        delete(tim);
    end

    % Send a final command and close the serial port
    write(serPort, 0.0, 'single');
    serPort.flush("input");
    clear serPort;
    fclose(datafileID);
    clear datafileID;

    logsout = readtable(FILEPATH, "VariableNamingRule","preserve","Delimiter",",");

    save(FILEPATH_MAT, "U_MAX", "U_MIN", "Y_SAFETY", "T_sample", "T_start", "T_stop", "u", "logsout");
    
    t = logsout.t;
    y = logsout.y;
    u = logsout.u;
    r = logsout.r;
    e = r - y;
    dt = logsout.dtp;
    
    
    figure(111);
    hold on;
    plot(t, y, '-k', 'LineWidth', 1.5);
    plot(t, r, '-r', 'LineWidth', 1.5);
    plot(t, u, '-b', 'LineWidth', 1.5);
    title('Step Response');
    % subtitle("P = " + num2str(P) + ", I = " + num2str(I) + ", D = " + num2str(D));
    legend('y(t)', 'ref(t)', 'u(t)', "Location", "best");
    xlabel('t [s]');
    ylabel('y [deg]');
    grid on;
    hold off;

end

[t, y, u, r] = runArduinoPlot();


%% Plot the data

figure(100);
subplot(3, 1, 1);
plot(t, y, t, r, 'LineWidth', 1.5);
grid minor;
legend('y','yhat','ref');
xlabel('t [s]');
ylabel('$\varphi [^\circ]$', 'Interpreter', 'latex');
title('System response');
subtitle("$\alpha - \beta$ filter", 'Interpreter', 'latex');
