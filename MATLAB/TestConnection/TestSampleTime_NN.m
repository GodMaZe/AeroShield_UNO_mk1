close all; clear;
clc;

addpath("./misc");

%% Prepare the environment for the measurement
DDIR = "dataRepo";
FILENAME = "dataFile";

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
Tstop = 20;

Ts = 0.02;
nsteps = floor(Tstop/Ts);

% Stop the measurement when the value of the output reaches or overtakes
% the following value
Ystop = 180; % deg

global LOG_T LOG_TP LOG_POT LOG_Y LOG_U LOG_DTP LOG_DT LOG_STEP LOG_CTRL_T LOG_REF;

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
LOG_X = [];

function plotdatarealtime()
    global LOG_T LOG_Y LOG_U LOG_REF;
    persistent hy hr hu;
    % ----------------------------------
    % Plot the measured data in real time
    % ----------------------------------
    try
        if isempty(hy) || isempty(hr) || isempty(hu)
            f = figure("Name","Plot-RealTime","GraphicsSmoothing","on","Renderer","opengl","RendererMode","auto");
            ax = axes(f);
            hold on;
            % hy = plot(ax, nan, nan, '.k');
            % hr = plot(ax, nan, nan, '.r');
            % hu = plot(ax, nan, nan, '.b');
            hy = stairs(ax, nan, nan);
            hr = stairs(ax, nan, nan);
            hu = stairs(ax, nan, nan);
            grid minor;
            title("Real-Time System Response");
            xlabel("t [s]");
            ylabel("$\varphi [^\circ]$", "Interpreter","latex");
            legend(ax, "y","ref", "u", 'Location', 'southeast');
            
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
        set(hu, 'YData', LOG_U(mask), 'XData', t);
        drawnow limitrate nocallbacks;
    catch err
       fprintf(2, "Plot thread: " + err.message + "\n");
    end
    % ----------------------------------
    % ----------------------------------
end

timerplotrealtime = timer('ExecutionMode','fixedRate', 'Period', 0.5, 'TimerFcn', @(~, ~) plotdatarealtime());
start(timerplotrealtime);


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

    sline = str2num(readline(scon));
    disp(sline);

    init_plant_time = -1;
    plant_output = sline(2);
    plant_input = sline(3);
    plant_potentiometer = sline(4);
    plant_dt = sline(5);
    plant_control_time = sline(6);

    time_start = datetime("now");
    time_curr = time_start;
    time_last = time_curr;

    plant_time_init = -1;
    plant_time = 0;
    step = 0;

    % REF = plant_potentiometer;

    % Load the neural network controller
    load("bestchrom_nn");

    y_max   = 220;           % ocakavane max(|y|)
    d1y_max = 50;           % ocakavane max(|dy/dt|)
    e_max   = 10;
    ie_max  = 10;    % odhad pre integral
    de_max = 100;
    d1u_max = 800;

    % y_max   = 1;           % ocakavane max(|y|)
    % d1y_max = 1;           % ocakavane max(|dy/dt|)
    % de_max = 1;
    % e_max   = 1;
    % ie_max  = 1;    % odhad pre integral
    % d1u_max = 1;

    Ny=1/y_max; Nd1y=1/d1y_max;
    Ne=1/e_max; Nie=1/ie_max; Nd1u=1/d1u_max; Nde=1/de_max;

    W1size = layers(1);
    W2size = layers(2);
    W3size = layers(3);

    W1 = reshape(bestchrom(1:W1size*W2size), W2size, W1size);
    W2 = reshape(bestchrom(W1size*W2size+1:W1size*W2size+W2size*W3size), W3size, W2size);
    W3 = reshape(bestchrom(W1size*W2size+W2size*W3size+1:end), 1, W3size);

    % Reference signal
    REF = 42;
    
    yinit = -1;
    dylast = 0;
    ylast= 0;
    ulast = 0;
    dulast = 0;
    elast = 0;
    eint = 0;

    e = 0;
    du = 0;

    U_PB = 30;
    u = 0;
    udt = 1;
    umax = 70;
    umin = -30;
    is_init = true;

    SYNC_TIME = 10;
    Tstop = Tstop + SYNC_TIME;
    nsteps = floor(Tstop/Ts);
    
    while plant_time < Tstop
        time_elapsed = seconds(time_curr - time_start);
        time_curr = datetime("now");
        time_delta = seconds(time_curr - time_last);

        if time_delta < Ts
            continue;
        end


        elapsed = time_elapsed - SYNC_TIME;

        if elapsed >= 15
            REF = 42;
        elseif elapsed >= 10
            REF = 37;
        elseif elapsed >= 5
            REF = 32;
        end

        % if elapsed >= 5
        %     REF = 5 * sin(0.5*elapsed) + 38;
        % end

        % REF = plant_potentiometer;
        
        if is_init
            u = u + udt;
            u = max(0, min(u, U_PB));
            if u >= U_PB
                is_init = false;
            end
        else
            u = U_PB;
        end
        
        
        
        if elapsed >= 0
            if yinit < 0
                yinit = ylast;
            end
            e = REF - ylast;
            de = (e - elast)/time_delta;
            eint = eint + e * time_delta;
        
            % X=[(ylast-yinit)*Ny; dylast*Nd1y; e*Ne; eint*Nie; de*Nde; dulast*Nd1u];
            X=[-dylast*Nd1y; e*Ne; eint*Nie; de*Nde; dulast*Nd1u];
            % writenum2file(fhandle, X);
            LOG_X = [LOG_X; X'];
            X = max(min(X,1),-1); % orezanie na interval <-1,1>
            % disp(X);
            
            if size(W1, 2) ~= length(X)
                error('Nespravny pocet vstupov do NC podla vah W1 a vektora X.');
            end
        
            % NN control law
            A1=(W1*X);    % vstupna/1.skryta vrstva
            A1=tanh(3*A1);
            A2=(W2*A1);   % 1./2. skryta vrstva
            A2=tanh(3*A2);
            ux=W3*A2*(umax-U_PB);
        
            u = u + min(umax-U_PB, max(-U_PB, ux));
        end

        du = (u - ulast)/time_delta;

        write(scon, u, "single");
        
        % Wait for the system to send a data message
        sline = str2num(readline(scon));

        if plant_time_init < 0
            plant_time_init = sline(1);
        end

        plant_time = sline(1) - plant_time_init;
        plant_output = sline(2);
        plant_input = sline(3);
        plant_potentiometer = sline(4);
        plant_dt = sline(5);
        plant_control_time = sline(6) - plant_time_init;

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

        step = step + 1;
        time_last = time_curr;

        dulast = du;
        dylast = (plant_output - ylast)/time_delta;
        ylast = plant_output;
        elast = e;
        ulast = u;

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
save(FILEPATH_MAT, "Tstop", "Ts", "nsteps", "logsout", "Ystop", "DDIR", "FILEPATH_MAT", "FILEPATH", "FILENAME");

%%
% ===========================
%   Plot Results
% ===========================

figure(1); clf;
hold on;
stairs(LOG_TP, LOG_Y);
stairs(LOG_TP, LOG_REF);
stairs(LOG_TP, LOG_U);
title("Real-Time System Response");
xlabel("t [s]");
ylabel("$\varphi [^\circ]$", "Interpreter","latex");
legend("y","ref", "u", 'Location', 'southeast');
grid minor;
hold off;


figure(999);
style='-k';

subplot(2,1,1)
hold on;
plot(LOG_TP, LOG_REF,"--k","LineWidth",1.5);
stairs(LOG_TP,LOG_Y,'LineWidth',1.5);
% scatter(LOG_TP, LOG_Y, '.k');
xlabel('k'); ylabel('\phi(k)'); grid on
% xlim([0,max(LOG_STEP)]);
hold of;

subplot(2,1,2)
stairs(LOG_TP,LOG_U,style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
% xlim([0,max(LOG_STEP)]);

set(gcf,'position',[200,400,650,400]);

figure(123); clf;
hold on;
for i=1:size(LOG_X, 2)
    subplot(size(LOG_X, 2), 1, i);
    plot(LOG_X(:, i));
    ylabel("X" + num2str(i));
    grid minor;
end
hold off;
