addpath("./genetic");
addpath("./misc");
clear;
close all;

%%
FILENAME_DATA = "data/train";
FILENAME = "train_ident_model";
FILENAME_TRAIN = "train_ident";

load(FILENAME);
load(FILENAME_DATA);

if ~exist(FILENAME_TRAIN, "dir")
    mkdir(FILENAME_TRAIN);
end

dt = mean(diff(logsout.tp));

% Set the system
Gs = Gs21; %GS21

% Gs = c2d(Gs, dt);

if exist('bestchrom_nn.mat', 'file')
    load('bestchrom_nn.mat');
    disp('Best chromosome loaded from file.');
end

%% Prepare reference signal


% t = 0:round(dt,2):5;
% r = ones(size(t))'*5;

t = 0:dt:20;
r = ones(size(t))'*5;
% r(1:120) = 20;
% r(121:end) = 1;
r(1:100) = 20;
r(100:250) = -15;
r(250:350) = 10;
r(350:500) = -20;
r(500:750) = 0;
r(750:850) = 5;
r(850:end) = -15;



%% Prepare Genetic
numgen=500;	% number of generations
lpop=150;	% number of chromosomes in population
W1size = 5;
W2size = 3;
W3size = 2;
layers = [W1size, W2size, W3size];
lstring=W1size * W2size + W2size * W3size + W3size;	% number of genes in a chromosome
M=1;          % maximum of the search space

NewPopsize = 10;
Randsize = 50;
WorkSize = (lpop - NewPopsize - Randsize - 2) / 2; % 2 for best selection

Space=[ones(1,lstring)*(-M); ones(1,lstring)*M];
Delta=Space(2,:)/5;    % additive mutation step
evolution = zeros(numgen, 1);

if ~exist('bestchrom', 'var')
    bestchrom = zeros(1, lstring);
end

if ~exist('best', 'var')
    best = 0;
end

bestgen = 0;

Pop = genrpop(lpop,Space);

%%
is_noise = true;
noise_amp = 0.012;


%% DO GENETIC
for gen=1:numgen
    Fit=fitness_NC(Pop,layers, Gs, r, t, is_noise, noise_amp);

    evolution(gen)=min(Fit);	% convergence graph of the solution

    Best = selbest(Pop, Fit, [1,1]);

    NewPop = genrpop(NewPopsize, Space);

    OldPop=selrand(Pop, Fit, Randsize);
    OldPop=mutx(OldPop, 0.10, Space);
    OldPop=muta(OldPop, 0.37, Delta, Space);

    Work1=seltourn(Pop,Fit,WorkSize);
    Work2=selsus(Pop,Fit,WorkSize);

    Work1=mutx(Work1, 0.15, Space);
    Work1=muta(Work1, 0.18, Delta, Space);

    Work2=muta(Work2,0.12,Delta,Space);
    Work2=mutx(Work2,0.15,Space);
    Work2=muta(Work2,0.18,Delta,Space);

    Pop=[Best;NewPop;OldPop;Work1;Work2];

    if best==0 || evolution(gen)<best
        best=evolution(gen);
        bestgen=gen;
        bestchrom=Best(1,:);
        fprintf('New best chromosome found in generation %d with fitness %.4f\n', bestgen, best);
    end

    if mod(gen, numgen/10) == 0
        x0 = zeros(numel(Gs.den) - 1, 1);
        [t,y,dy,w,e,de,u,du]=sim_ncFF_VIR25(...
            reshape(bestchrom(1:W1size*W2size), W2size, W1size), ...
            reshape(bestchrom(W1size*W2size+1:W1size*W2size+W2size*W3size), W3size, W2size), ...
            reshape(bestchrom(W1size*W2size+W2size*W3size+1:end), 1, W3size), ...
            Gs, r, t, x0, -100, 100, [], is_noise, noise_amp);
        figure(2); clf;
        hold on;
        stairs(t, y, 'b');
        stairs(t, w, 'r--');
        stairs(t, u);
        title('Output and Reference Signal');
        subtitle("Gen = " + num2str(gen));
        xlabel('t [s]');
        ylabel('response [-]');
        legend('y', 'r','u');
        grid on;
        hold on;
        saveas(gcf, FILENAME_TRAIN + "/output_nn_best_train_gen_" + num2str(gen) + ".png");
    end
    fprintf('Generation %d completed.\n', gen);
end

save(FILENAME_TRAIN + "/bestchrom_nn.mat", 'bestchrom', 'layers', 'best', "evolution");

%% Save and Draw Best
close all;
if ~exist(FILENAME_TRAIN + "/bestchrom_nn.mat", 'file')
    if ~exist('layers', 'var') || ~exist('bestchrom', 'var') || ~exist('evolution', 'var')
        error('Variables "layers", "rezim", "bestchrom", and "evolution" must be defined in the workspace.');
    end
end

load(FILENAME_TRAIN + "/bestchrom_nn.mat");

W1size = layers(1);
W2size = layers(2);
W3size = layers(3);


x0 = zeros(numel(Gs.den) - 1, 1);

[t,y,dy,w,e,de,u,du]=sim_ncFF_VIR25(...
    reshape(bestchrom(1:W1size*W2size), W2size, W1size), ...
    reshape(bestchrom(W1size*W2size+1:W1size*W2size+W2size*W3size), W3size, W2size), ...
    reshape(bestchrom(W1size*W2size+W2size*W3size+1:end), 1, W3size), ...
    Gs, r, t, x0, -100, 100, [], is_noise, noise_amp);

figure(1);
plot(evolution);
title('Evolution of Neural Network Controller');
xlabel('Generation');
ylabel('Fitness');
grid on;
saveas(gcf, FILENAME_TRAIN + "/evolution_nn.png");

figure(2);
hold on;
stairs(t, y, 'b');
stairs(t, w, 'r--');
stairs(t, u);
title('Output and Reference Signal');
subtitle("Train");
xlabel('Time [s]');
ylabel('Amplitude');
legend('y', 'r','u');
grid on;
hold on;
saveas(gcf, FILENAME_TRAIN + "/output_nn_best_train.png");

%% Test Trained
x0 = zeros(numel(Gs.den) - 1, 1);
t = 0:dt:20;
r = ones(size(t))';
r(1:100) = 5;
r(100:200) = -5;
r(200:300) = 0;
r(300:end) = 4;

if ~exist('layers', 'var') || ~exist('bestchrom', 'var') || ~exist('evolution', 'var')
    error('Variables "layers", "rezim", "bestchrom", and "evolution" must be defined in the workspace.');
end

W1size = layers(1);
W2size = layers(2);
W3size = layers(3);

[t,y,dy,w,e,de,u,du]=sim_ncFF_VIR25(...
    reshape(bestchrom(1:W1size*W2size), W2size, W1size), ...
    reshape(bestchrom(W1size*W2size+1:W1size*W2size+W2size*W3size), W3size, W2size), ...
    reshape(bestchrom(W1size*W2size+W2size*W3size+1:end), 1, W3size), ...
    Gs, r, t, x0, -100, 100, [], is_noise, noise_amp);
%%
try
    close(3);
catch er
end
figure(3);
hold on;
stairs(t, y);
stairs(t, w, 'k--');
stairs(t, u, 'r')
title('Output and Reference Signal');
subtitle("Test");
xlabel('Time [s]');
ylabel('Amplitude');
legend('y', 'r', 'u');
grid on;
saveas(gcf, FILENAME_TRAIN + "/output_nn_best_test.png");
hold off;