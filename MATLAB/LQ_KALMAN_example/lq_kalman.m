format short;

DDIR = "dataRepo";
if ~exist(DDIR, "dir")
    fprintf("Creating the data directory...");
    mkdir(DDIR);
end


% ----------------------------------
% ----------------------------------

% Define time parameters

T_start = 0;

T_sample = 50;      % [s] <1, 255>

% Define STOP TIME

T_stop = 60.0;     % [sec]



% Define control parameters
U_MAX = 100.0;
U_MIN = 0.0;
Y_SAFETY = 190.0;

R_WANTED = 20;

% ----------------------------------
% ----------------------------------

% Define model parameters
K = 1.3860;
eta = 11.0669;
omega = 7.8944;
b = 0.0735;

% State matrix A
Ac = [-eta, 0, 0;
      0, 0, 1;
      omega^2, -omega^2, -2*b*omega];

% Input matrix B
Bc = [K*eta; 0; 0];

% Output matrix C
Cc = [0, 1, 0];

sys = ss(Ac,Bc,Cc,0);
sysd = c2d(sys, T_sample/1000);

[A, B, C, ~] = ssdata(sysd);

function [timer_t, timer_y, timer_u, timer_potentiometer, timer_yhat, timer_dyhat] = runArduinoPlot(A, B, C, dirpath, Ts, Tstart, Tstop, umin, umax, ysafe, ref)
    DDIR = dirpath;
    T_sample = Ts;
    T_stop = Tstop;
    T_start = Tstart;
    U_MAX = umax;
    U_MIN = umin;
    Y_SAFETY = ysafe;
    R_WANTED = ref;
    %%
    n = size(A, 1);
    r = size(B, 2);
    m = size(C, 1);

    % --- Augmented system for integral action ---
    A_tilde=zeros(n+m,n+m);
    B_tilde=zeros(n+m, r);
    
    A_tilde = [A, zeros(n, m);
               -C, eye(m, m)];

    % A_tilde(1:n,1:n) = A;
    % A_tilde(n+1:end,1:n) = -C;
    % A_tilde(n+1:end,n+1:end) = eye(m, m);
    
    B_tilde(1:n) = B;

    % --- LQ weighting matrices ---
    Q_=[0.1 0 0;
        0 1 0;
        0 0 0.25];
    R_=[0.1];
    Qz=[1];
    Q_tilde=zeros(size(Q_) + size(Qz));
    Q_tilde(1:n, 1:n) = Q_;
    Q_tilde(n+1:end, n+1:end) = Qz;
    
    % --- Solve Discrete-time Algebraic Riccati Equation ---
    [P_LQ,~,K_LQ]= dare(A_tilde, B_tilde, Q_tilde, R_);
    K_LQ = -K_LQ;
    
    Kx=K_LQ(1:n);           % state feedback part
    Kz=K_LQ(n + 1:end);        % integral feedback part

    % --- Kalman filter initialization ---    
    R=0.3; % measurement noise covariance
    Q=diag([0.2;0.2;1]);  % process noise covariance

    %% ----------------------------------
    % ----------------------------------
    
    timer_t = [];
    timer_y = [];
    timer_yhat = [];
    timer_dyhat = [];
    timer_u = [];
    timer_potentiometer = [];
    
    % Data save

    for tim=timerfindall
        stop(tim);
        delete(tim);
    end
    
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
            set(hu, 'YData', timer_yhat, 'XData', timer_t);
            drawnow limitrate nocallbacks;
        catch err
           fprintf(2, "Plot thread: " + err.message + "\n");
        end
        % ----------------------------------
        % ----------------------------------
    end
    
    tPlot = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotData());
    start(tPlot);
    
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
    
    
    % Write data into files
    
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
    plant_potentiometer = R_WANTED + serLineList(4)/100*20;
    plant_dt = serLineList(5);

    timer_yhat = [timer_yhat, 0];
    timer_dyhat = [timer_dyhat, 0];
    
    % Display the received data
    tmp_printlist = [0, plant_time, plant_potentiometer, plant_output, plant_input, plant_dt, T_sample];
    doUpdate(tmp_printlist);
    
    
    % Kalman initial
    P=zeros(n);
    x_hat=zeros(n,1);
        
    % Initialize control variables
    z=zeros(m,1);
    u=zeros(r,1);
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
        plant_potentiometer = R_WANTED + serLineList(4)/100*20; % R_WANTED; %*sin(2*pi*20*plant_time/1e6) + R_WANTED; % serLineList(4);
        plant_dt = serLineList(5);
        
        x_hat = A*x_hat + B*u;

        P = A*P*A' + Q;
        K = P*C'/(C*P*C' + R);
        e = plant_output - C*x_hat;
        x_hat = x_hat + K*e;
        y_hat = C*x_hat;
        P = P - K*C*P;

        timer_yhat = [timer_yhat, y_hat];
        timer_dyhat = [timer_dyhat, x_hat(1)];

        % Display the received data
        tmp_printlist = [time_elapsed, plant_time, plant_potentiometer, plant_output, plant_input, plant_dt, time_delta];
        doUpdate(tmp_printlist);
    
    
        u = Kx*x_hat + Kz*z;
    
        % u=plant_potentiometer;
    
        u_send = u;

        u_send = min(U_MAX, max(u_send, U_MIN));

        z = z + plant_potentiometer - plant_output;
        % z = z + 0.7*e + (plant_potentiometer - plant_output)*(0.6 + 0.8/(plant_dt/1000));

        % if u_send > U_MAX
        %     u_send = U_MAX;
        % elseif u_send < U_MIN
        %     u_send = U_MIN;
        % end
    
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
    title('Control Response');
    legend('y(t)', 'ref(t)', 'u(t)', "Location", "best");
    xlabel('t [s]');
    ylabel('y [deg]');
    grid on;
    hold off;

end

[t, y, u, potentiometer, yhat, dyhat] = runArduinoPlot(A, B, C, DDIR, T_sample, T_start, T_stop, U_MIN, U_MAX, Y_SAFETY, R_WANTED);


%% Plot the data

figure(100);
subplot(3, 1, 1);
plot(t, y, t, yhat, t, potentiometer, 'LineWidth', 1.5);
grid minor;
legend('y','yhat','ref');
xlabel('t [s]');
ylabel('$\varphi [^\circ]$', 'Interpreter', 'latex');
title('System response');
subtitle("Kalman filter", 'Interpreter', 'latex');

subplot(3, 1, 2);
plot(t, u, 'LineWidth', 1.5);
grid minor;
xlabel('t [s]');
ylabel('$\omega [^\circ/s]$', 'Interpreter', 'latex');
title('System velocity response');
subtitle("Kalman filter", 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t, (y-yhat), 'LineWidth', 1.5);
grid minor;
xlabel('t [s]');
ylabel('$\varphi [^\circ]$', 'Interpreter', 'latex');
title('Observer error');
subtitle("Kalman filter", 'Interpreter', 'latex');

%% Test
tt = 0:T_sample/1000:t(end);
ut = interp1(t, u, tt);
[yx, tx] = lsim(sysd, u, tt(1:numel(u)));

figure(1234);
hold on;
plot(tx, yx, 'DisplayName', 'yinterp');
plot(t, y, 'DisplayName', 'yreal');
grid minor;
legend show;
hold off;