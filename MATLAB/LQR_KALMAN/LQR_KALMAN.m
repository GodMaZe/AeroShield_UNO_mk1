%% 
close all; clear;
clc;

addpath("../misc");
addpath("../misc/KF");
addpath("../misc/models");
addpath("../misc/functions");
addpath("../misc/models/frictions");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "lqrkalman";

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
SYNC_TIME = 10; % Time for the system to stabilize in the OP

Ts = 0.05;
nsteps_solo = floor(Tstop/Ts);

Tstop = Tstop + SYNC_TIME;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg
UMax = 70;
UMin = -30;

global LOG_T LOG_TP LOG_POT LOG_Y LOG_U LOG_DTP LOG_DT LOG_STEP LOG_CTRL_T LOG_REF LOG_YHAT LOG_XHAT;

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
LOG_YHAT = [];
LOG_XHAT = [];
LOG_UX = [];

function plotdatarealtime()
    global LOG_T LOG_Y LOG_YHAT LOG_REF;
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
            hr = stairs(ax, nan, nan, '--k');
            hu = stairs(ax, nan, nan);
            grid minor;
            title("Real-Time System Response");
            xlabel("t [s]");
            ylabel("$\varphi [^\circ]$", "Interpreter","latex");
            legend(ax, "y","ref", "yhat", 'Location', 'southwest');
            
        end
       
        % plot(plot_t, plot_sig_3,'.b', plot_t, plot_sig_2,'.r', plot_t,
        % plot_sig_1,'.k')
        % print(timer_t(1));
        nsteps = numel(LOG_Y);
        last_n_points = nsteps - 300;

        if last_n_points < 0
            last_n_points = 1;
        end

        mask = last_n_points:nsteps;
        t = LOG_T(mask);

        set(hy, 'YData', LOG_Y(mask), 'XData', t);
        set(hr, 'YData', LOG_REF(mask), 'XData', t);
        set(hu, 'YData', LOG_YHAT(mask), 'XData', t);
        drawnow limitrate nocallbacks;
    catch err
       fprintf(2, "Plot thread: " + err.message + "\n");
    end
    % ----------------------------------
    % ----------------------------------
end

%% Init model, LQR, Kalman
% ----------------------------------
% ----------------------------------

% % Define model parameters
% K = 1.3860;
% eta = 11.0669;
% omega = 7.8944;
% b = 0.0735;
% 
% % State matrix A
% Ac = [-eta, 0, 0;
%       0, 0, 1;
%       omega^2, -omega^2, -2*b*omega];
% 
% % Input matrix B
% Bc = [K*eta; 0; 0];
% 
% % Output matrix C
% Cc = [0, 1, 0];
% 
% sys = ss(Ac,Bc,Cc,0);
% sysd = c2d(sys, Ts);
    
% Initialize control variables
z=0;
u=0;
u_send = u;

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

    scon = serialport("COM3", 250000, "Timeout", 5);
    
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
    time_step = time_curr;

    plant_time_init = -1;
    plant_time = 0;
    

    % REF_INIT = 35;
    REF_INIT = 30;
    REF = REF_INIT;

    REF_STEPS = [-10, -5, 0, 10, -REF_INIT];
    % REF_STEPS(:) = -REF_INIT;
    nsteps_per_ref = floor(nsteps_solo / (numel(REF_STEPS) + 1));

    U_STEP_SIZE = 5;
    
    U_PB = 30;

    step = 0;
    step_init = 0;

    u = 0;
    ux = 0;
    udt = U_STEP_SIZE/10;
    is_init = true;
    e = 0;

    %% Init system and Kalman Filter
    % [A, B, C, ~] = ssdata(sysd);
    % pendulum = Pendulum();
    load("../misc/models/ipendulum_model");
    pendulum = sys;
    % pendulum.Ku = pendulum.Ku/0.72;
    % [A, B, C, D] = pendulum.ss_discrete(Ts);
    [pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts);
    
    
    if exist("pendulum", "var")
        n = pendulum.n;
        m = pendulum.m;
        r = pendulum.r;
    else
        n = size(A, 1);
        r = size(B, 2);
        m = size(C, 1);   
    end
    
    x0 = zeros(n, 1);
    
    % --- Augmented system for integral action ---
    if exist("Fx","var") && exist("Hx","var") && exist("Bu", "var")
        A = discrete_jacobian(f, 0, x0, 0, Ts);
        C = Hx(0, x0, 0);
        B = discrete_jacobian_u(f, 0, x0, 0, Ts);
    end
    B_tilde=zeros(n+m, r);
    
    A_tilde = [A, zeros(n, m);
               -C, eye(m, m)];
    
    B_tilde(1:n) = B;
    
    % --- LQ weighting matrices ---
    % Q_=[0.01 0 0;
    %     0 10 0;
    %     0 0 7];

    Q_=diag([1 5]);
    R_=[0.1];
    Qz=[15];

    Q_tilde=[Q_, zeros(size(Q_, 1), size(Qz, 2));
            zeros(size(Qz, 1), size(Q_, 2)), Qz];

    % --- Solve Discrete-time Algebraic Riccati Equation ---
    [P_LQ,~,K_LQ] = dare(A_tilde, B_tilde, Q_tilde, R_);
    K_LQ = -K_LQ;
    
    Kx=K_LQ(1:n);           % state feedback part
    Kz=K_LQ(n + 1:end);        % integral feedback part
    
    
    % --- Kalman filter initialization ---    
    % R = (0.015); % Measurement noise (from datasheet)
    % Q = diag(([0.001 (0.001*Ts)]));

    % R = (0.015)^2; % Measurement noise (from datasheet)
    % Q = diag([(0.01)^2 (0.01/Ts)^2]);

    [Q, R] = QR_matrix(n, m);
    
    % R = 3;
    % Q = diag([0.1 0.1 1]);
    
    % Kalman initial
   

    P = diag(ones(size(x0))*var(x0));
    x_hat = x0;
    y_hat = h(0, x_hat, 0);

    % KF = KalmanFilter(A, B, C, 'R', R, 'Q', Q, 'x0', x_hat);
    EKF = ExtendedKalmanFilter(f, h, x_hat, 1, 'Q', Q, 'R', R, 'P0', P, 'epstol', Ts);

    %% Loop
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if step > 0 && time_delta < Ts
            continue;
        end

        elapsed = time_elapsed - SYNC_TIME;

        if elapsed >= 0
            if step_init == 0
                step_init = step;
                istep = 1;
            else
                istep = istep + 1;
            end
            
            if mod(istep, nsteps_per_ref) == 0 && istep/nsteps_per_ref <= numel(REF_STEPS)
                REF = REF_INIT + REF_STEPS(istep/nsteps_per_ref);
            end

            % if exist("Fx", "var")
            %     x_test = x_hat; %[deg2rad(aerodata.output); x_hat(2)];
            %     A_new = discrete_jacobian(f, plant_time, x_test, u, Ts);
            %     C_new = Hx(plant_time, x_test, u);
            %     B_new = discrete_jacobian_u(f, plant_time, x_test, u, Ts);
            % 
            %     % A = (A+A_new)/2;
            %     % B = (B+B_new)/2;
            %     % C = (C+C_new)/2;
            % 
            %     A = A_new;
            %     B = B_new;
            %     C = C_new;
            % 
            %     A_tilde(1:size(A, 1), 1:size(A, 2)) = A;
            % 
            %     B_tilde(1:n, :) = B; 
            % 
            %     % --- Solve Discrete-time Algebraic Riccati Equation ---
            %     [P_LQ,~,K_LQ] = dare(A_tilde, B_tilde, Q_tilde, R_);
            %     K_LQ = -K_LQ;
            % 
            %     Kx=K_LQ(1:n);           % state feedback part
            %     Kz=K_LQ(n + 1:end);        % integral feedback part
            % end

            ux = Kx*x_hat + Kz*z;
            e = deg2rad(REF) - y_hat; % EKF
            z = z + e;

            u = U_PB + saturate(ux, -U_PB, 100 - U_PB);
            % u = saturate(ux, 0, 100);
        else
            u = U_PB;
        end


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

        % The problem is with the input matrix B. It is too large of an
        % input for the simple pendulum, the control input should be scaled
        % down as it represents directly the torque applied to the arm, not
        % the %PWM value.
        [EKF, y_hat] = EKF.step(plant_time, u, deg2rad(aerodata.output));
        y_hat = y_hat;
        x_hat = EKF.get_xhat();

        % [KF, y_hat] = KF.step(u, deg2rad(aerodata.output));
        % y_hat = y_hat;
        % x_hat = KF.get_xhat();

        % [KF, y_hat] = KF.step(u, aerodata.output);
        % y_hat = y_hat;
        % x_hat = KF.get_xhat();

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
        LOG_YHAT = [LOG_YHAT, rad2deg(y_hat)];
        LOG_XHAT = [LOG_XHAT; rad2deg(x_hat)'];
        LOG_UX = [LOG_UX, ux];

        time_last = time_curr;
        step = step + 1;


        if plant_time >= Tstop || plant_output >= Ystop
            % configureCallback(scon, "off"); % Remove the callback from the serial port, before exiting the loop
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
save(FILEPATH_MAT, "Tstop","SYNC_TIME","U_PB", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME", "U_STEP_SIZE");

%%
% ===========================
%   Plot Results
% ===========================
figure(1); clf;
hold on;
stairs(LOG_TP, LOG_Y);
stairs(LOG_TP, LOG_REF);
stairs(LOG_TP, LOG_YHAT);
title("Real-Time System Response");
xlabel("t [s]");
ylabel("$\varphi [^\circ]$", "Interpreter","latex");
legend("y","ref", "yhat", 'Location', 'southeast');
grid minor;
hold off;

figure(999); clf;
style='-k';

subplot(2,1,1)
hold on;
plot(LOG_TP, LOG_REF,"--k","LineWidth",1.5);
stairs(LOG_TP,LOG_Y,'LineWidth',1.5);
stairs(LOG_TP, LOG_YHAT,'LineWidth',1.5);
xlabel('t [s]'); ylabel('\phi(k)'); grid on;
% xlim([0,max(LOG_STEP)]);
legend("ref","y","yhat");
hold off;

subplot(2,1,2)
hold on
stairs(LOG_TP, LOG_U,'LineWidth',1.5);
stairs(LOG_TP, LOG_UX,'LineWidth',1.5);
ylabel('u(t) [%]'); xlabel('t [s]'); grid on
% xlim([0,max(LOG_STEP)]);
legend("u","ux");
hold off
set(gcf,'position',[200,400,650,400]);

%%
figure(123); clf;
hold on;
for i=1:size(LOG_XHAT, 2)
    subplot(size(LOG_XHAT, 2), 1, i);
    plot(LOG_XHAT(:, i));
    ylabel("X" + num2str(i));
    grid minor;
end
hold off;

%% Plot control error
tidx = 1; % find(LOG_TP >= 5, 1);
tmask = tidx:numel(LOG_REF);
mLOG_STEP = LOG_STEP(tmask);
mLOG_STEP = mLOG_STEP - mLOG_STEP(1);
mLOG_TP = LOG_TP(tmask);
mLOG_TP = mLOG_TP - mLOG_TP(1);
mLOG_REF = LOG_REF(tmask);
mLOG_Y   = LOG_Y(tmask);
LOG_E = mLOG_REF - mLOG_Y;
emean = mean(LOG_E);
sLOG_E = LOG_E - emean;
estd = std(sLOG_E);
evar = estd^2;

figure(24); clf;
hold on;
plot(mLOG_TP, sLOG_E);
plot([mLOG_TP(1), mLOG_TP(end)], [0, 0]);
title("Control error");
subtitle("Mean square error: " + num2str(emean^2))
grid minor;
hold off;
fprintf("Statistical data:\n Mean: %8.3f\n Var: %8.3f\n STD: %8.3f\n", emean, estd, evar);

%% Plot the frequency analysis (fft) of the control error
mkdir("figures");
freq = (mLOG_STEP)*1/Ts/numel(mLOG_STEP);
yf = fft(sLOG_E);

figure(31); clf;
semilogx(log10(freq), 20*log10(abs(yf)))
xlabel("f [Hz]");
ylabel("|A| [db]");
title("Frequency characteristic");
subtitle("Control Error");
grid minor;
grid on;
saveas(gcf, "figures/lqr_control_error_freq_char.fig", "fig");

[pxx,f] = periodogram(sLOG_E,[],[],1/Ts);
figure(32); clf;
semilogx(log10(f),pxx);
title("Periodicity");
subtitle("Control Error");
ylabel("Magnitude [-]");
xlabel("f [Hz]");
grid minor;
grid on;
saveas(gcf, "figures/lqr_control_error_periodicity.fig", "fig");
