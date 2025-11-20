close all; clear;
clc;

addpath("./misc");
addpath("./misc/MPC");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "mpckalman";

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
SYNC_TIME = 20; % Time for the system to stabilize in the OP

Ts = 0.05;
p = 20; % Prediction horizon

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

timerplotrealtime = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotdatarealtime());
start(timerplotrealtime);


%% Init model, MPC, Kalman
% ----------------------------------
% ----------------------------------
options = optimoptions('quadprog', ...
                       'ConstraintTolerance', 1e-5, ...
                       'OptimalityTolerance',1e-5, ...
                       'Display', 'off');

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
sysd = c2d(sys, Ts);

[A, B, C, ~] = ssdata(sysd);

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

%% --- MPC prediction matrices ---
[M,N] = mpcfillmnoutput(A_tilde,B_tilde,C_tilde,p);  % build prediction matrices
[Gamma] = mpcfillgamma(r,p);

% --- MPC weighting matrices ---
% Q_mpc = [1];
% R_mpc = [10];
Q_mpc=[0.01];
R_mpc=[0.1];

Q_ = diagblock(Q_mpc, p);
R_ = diagblock(R_mpc, p);

% Quadratic cost Hessian
H = 2*(Gamma'*N'*Q_*N*Gamma + R_);
H = (H+H')/2; % for symmetry

% Control input constraints
u_lower = [UMin];
u_upper = [UMax];

U_lower = repmat(u_lower, p, 1);
U_upper = repmat(u_upper, p, 1);

y_lower = [-50];
y_upper = [80];

Y_lower = repmat(y_lower, p, 1);
Y_upper = repmat(y_upper, p, 1);

% Constraint matrix for quadratic programming
A_con = [Gamma;
        -Gamma;
         N*Gamma;
        -N*Gamma];

% --- Kalman filter initialization ---    
R=0.01; % measurement noise covariance
Q=diag([0.1;0.1;0.1;0.1]);  % process noise covariance

% Kalman initial
P=zeros(size(Q));
x_hat=zeros(size(Q,1),1);

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
        scon.flush("input");
        clear scon;
    end

    scon = serialport("COM3", 115200, "Timeout", 5);
    
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

        if time_delta < Ts
            continue;
        end

        % if is_init
        %     u = u + udt;
        %     u = max(0, min(u, U_PB));
        %     if u >= U_PB
        %         is_init = false;
        %     end
        % else
        %     u = U_PB;
        % end

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

        % Do Kalman
        x_hat = A_tilde*x_hat + B_tilde*u;

        P = A_tilde*P*A_tilde' + Q;
        K = P*C_tilde'/(C_tilde*P*C_tilde' + R);
        e1 = plant_output - C_tilde*x_hat;
        x_hat = x_hat + K*e1;
        y_hat = C_tilde*x_hat;
        P = P - K*C_tilde*P;
        % End Kalman

        
        
        if elapsed >= 0
            % Do MPC
            u_pred = u_ones*u;

            Y_ref = repmat(REF, p, 1);

            b = 2*(M*x_hat + N*u_pred - Y_ref)'*Q_*N*Gamma;
            b_con = [U_upper - u_pred;
                    -U_lower + u_pred;
                    Y_upper - M*x_hat - N*u_pred;
                    -Y_lower + M*x_hat + N*u_pred];

            tic
            delta_U = quadprog(H, b, A_con, b_con, [], [], [], [],[], options);
            toc
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

        % Write the data into a file
        data = [time_elapsed, plant_time, plant_output, plant_input, plant_potentiometer, plant_dt, time_delta, plant_control_time, REF];
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
        LOG_YHAT = [LOG_YHAT, y_hat];
        LOG_XHAT = [LOG_XHAT; x_hat'];
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
    if exist("scon", "var")
        write(scon, 0.0, 'single');
        clear scon;
    end
    if exist("dfile_handle", "var")
        fclose(dfile_handle);
        clear dfile_handle;
    end
    for tim=timerfindall
        stop(tim);
        delete(tim);
    end
    rethrow(er);
end

%% close conns
if exist("scon", "var")
    write(scon, 0.0, 'single');
    clear scon;
end
if exist("dfile_handle", "var")
    fclose(dfile_handle);
    clear dfile_handle;
end
for tim=timerfindall
    stop(tim);
    delete(tim);
end

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
