close all; clear;
clc;

addpath("../misc");
addpath("../misc/MPC");
addpath("../misc/functions");
addpath("../misc/models");
addpath("../misc/KF");

% Script for loading all the system-wide matlab configurations
loadconfigs;

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "mpc_ekf_state";

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
Tstop = 10;
SYNC_TIME = 5; % Time for the system to stabilize in the OP

Ts = 0.02;
p = 15; % Prediction horizon

U_PB = 30;

Tstop = Tstop + SYNC_TIME;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg
UMax = 100;
UMin = 0;

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

%% Init model, MPC, Kalman
% ----------------------------------
% ----------------------------------
options = optimoptions("quadprog", ...
                       'ConstraintTolerance', 1e-5, ...
                       'OptimalityTolerance',1e-5, ...
                       'Display', 'off');


pendulum = Pendulum();

[A, B, C, ~] = pendulum.ss_discrete(Ts);

n = size(A, 1);
r = size(B, 2);
m = size(C, 1);

% --- Augmented system for integral action ---
D = [1]; % Disturbance (the discrepancy between the model and real system)

d = size(D, 1);

A_tilde = [A, zeros(n, d);
           zeros(d, n), D];
B_tilde = [B; zeros(r, d)];

C_tilde = [C, eye(d)];

A_tilde = A;
B_tilde = B;
C_tilde = C;



% --- MPC weighting matrices ---
Q_mpc_=[15 0;
        0 0.75];
Q_mpc_=[1];
R_mpc=[0.6];
Qd_mpc = [0.001];

Q_mpc = [Q_mpc_ zeros(size(Q_mpc_, 1), size(Qd_mpc, 2));
        zeros(size(Qd_mpc, 1), size(Q_mpc_, 2)), Qd_mpc];

Q_mpc = Q_mpc_;



Q_ = diagblock(Q_mpc, p);
R_ = diagblock(R_mpc, p);

%% --- MPC prediction matrices ---
[M,N] = mpcfillmnoutput(A_tilde,B_tilde,C_tilde,p);  % build prediction matrices
[Gamma] = mpcfillgamma(r,p);
% Quadratic cost Hessian
H = 2*(Gamma'*N'*Q_*N*Gamma + R_);
% H = 2*(N'*Q_*N + R_);
H = (H+H')/2; % for symmetry

% Control input constraints
u_lower = [UMin];
u_upper = [UMax];

U_lower = repmat(u_lower, p, 1);
U_upper = repmat(u_upper, p, 1);

x_lower = [-100; -300];% -100];
x_upper = [100; 300];% 100];

X_lower = repmat(x_lower, p, 1);
X_upper = repmat(x_upper, p, 1);

y_lower = [-pi/3];
y_upper = [7*pi/6];

Y_lower = repmat(y_lower, p, 1);
Y_upper = repmat(y_upper, p, 1);

% Constraint matrix for quadratic programming
I = eye(size(Gamma));
A_con = [Gamma;
        -Gamma;
         N*Gamma;
        -N*Gamma];

%% --- Kalman filter initialization --- 
[pendulum, f, b, h, Fx, Bu, Hx] = pendulum.nonlinear(Ts, false);

[Q, R] = QR_matrix(n, m);

x0 = [0; 0];
P = diag(ones(size(x0))*var(x0));

ekf = ExtendedKalmanFilter(f,h,x0,1,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'P0',P,"epstol",Ts);

kf = KalmanFilter(A_tilde,B_tilde,C_tilde,'Q',Q,'R',R,'x0',x0,'P0',P);

x_hat = x0;
%% Try connecting and running the measurement
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

    scon = serialport("COM3", CF_BAUDRATE, "Timeout", CF_TIMEOUT);
    
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
    

    REF_INIT = 35;
    REF = REF_INIT;

    REF_STEPS = [5, -5, 0, 10, -REF_INIT];

    U_STEP_SIZE = 5;
    

    step = 1;
    u = U_PB;
    ux = 0;
    udt = U_STEP_SIZE/10;
    is_init = true;
    e = 0;
    

    u_ones = ones(p, r);
    
    
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if step > 0 && time_delta < Ts
            continue;
        end

        elapsed = time_elapsed - SYNC_TIME;

        if elapsed >= 25
            REF = REF_INIT + REF_STEPS(5);
        elseif elapsed >= 20
            REF = REF_INIT + REF_STEPS(4);
        elseif elapsed >= 15
            REF = REF_INIT + REF_STEPS(3);
        elseif elapsed >= 10
            REF = REF_INIT + REF_STEPS(2);
        elseif elapsed >= 5
            REF = REF_INIT + REF_STEPS(1);
        end        
        
        if elapsed >= 0
            % Do MPC
            % [M,N] = mpcfillmnstate(Fx(x_hat,u),[0;1/pendulum.I_T;0],p);  % build prediction matrices
            % % Quadratic cost Hessian
            % H = 2*(Gamma'*N'*Q_*N*Gamma + R_);
            % H = (H+H')/2; % for symmetry
            u_pred = u_ones*u;

            % x_test = x_hat;
            % x_test(1) = deg2rad(REF) - x_test(3);
            % x_test(3) = 0;
            
            Y_ref = repmat(deg2rad(REF), p, 1);
            
            Mx = M*(x_hat);
            Nu = N*u_pred;
            b = 2*(Mx + Nu - Y_ref)'*Q_*N*Gamma;
            b_con = [U_upper - u_pred;
                    -U_lower + u_pred;
                    Y_upper - Mx - Nu;
                    -Y_lower + Mx + Nu];

            % tic
            delta_U = quadprog(H, b, A_con, b_con, [], [], [], [], [], options);
            % toc
            if ~isempty(delta_U)
                ux = delta_U(1:r, :);
            end
            u = u + ux;
            % End MPC
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

        % Do Kalman
        % [ekf, y_hat] = ekf.step(plant_time, u/666.66, deg2rad(plant_output));
        % x_hat = ekf.xhat;
        [kf, y_hat] = kf.step(u, deg2rad(plant_output));
        x_hat = kf.xhat;
        % y_hat = rad2deg(y_hat);

        % x_hat = A_tilde*x_hat + B_tilde*u;
        % 
        % P = A_tilde*P*A_tilde' + Q;
        % S = (C_tilde*P*C_tilde' + R);
        % K = P*C_tilde'/S;
        % e1 = plant_output - C_tilde*x_hat;
        % x_hat = x_hat + K*e1;
        % y_hat = C_tilde*x_hat;
        % P = P - K*C_tilde*P;
        % End Kalman

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
legend("ref","y","yhat");
xlabel('k'); ylabel('\phi(k)'); grid on;
% xlim([0,max(LOG_STEP)]);
hold off;

subplot(2,1,2)
hold on
stairs(LOG_TP, LOG_U,'LineWidth',1.5);
stairs(LOG_TP, LOG_UX,'LineWidth',1.5);
legend("u","ux");
ylabel('u(k) [%]'); xlabel('t [s]'); grid on

% xlim([0,max(LOG_STEP)]);
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
saveas(gcf, "figures/state_mpc_control_error_freq_char.fig", "fig");

[pxx,f] = periodogram(sLOG_E,[],[],1/Ts);
figure(32); clf;
semilogx(log10(f),pxx);
title("Periodicity");
subtitle("Control Error");
ylabel("Magnitude [-]");
xlabel("f [Hz]");
grid minor;
grid on;
saveas(gcf, "figures/state_mpc_control_error_periodicity.fig", "fig");
