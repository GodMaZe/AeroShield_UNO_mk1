addpath("./genetic");
clear;
close all;


FILENAME_DATA = "data/train";
FILENAME = "train_ident_model";
FILENAME_TRAIN = "train_ident";

load(FILENAME);
load(FILENAME_DATA);

if ~exist(FILENAME_TRAIN, "dir")
    mkdir(FILENAME_TRAIN);
end

% Set the system
Gs = Gs20; %GS21

if exist('bestchrom_pid.mat', 'file')
    load('bestchrom_pid.mat');
    disp('Best chromosome loaded from file.');
end

%% START GA

numgen=50;	% number of generations
lpop=150;	% number of chromosomes in population
lstring=3;	% number of genes in a chromosome
M=[1, 8, 1];          % maximum of the search space

NewPopsize = 15;
Randsize = 41;
WorkSize = (lpop - NewPopsize - Randsize - 2) / 2; % 2 for best selection

Space=[zeros(1,lstring); ones(1,lstring).*M];
Delta=Space(2,:)/5;    % additive mutation step
evolution = zeros(numgen, 1);

Pop = genrpop(lpop,Space/10);

if ~exist('bestchrom', 'var')
    bestchrom = zeros(1, lstring);
else
    Pop(1,:) = bestchrom;
end

if ~exist('best', 'var')
    best = 0;
end

bestgen = 0;

dt = mean(diff(logsout.tp));
t = 0:dt:20;
r = ones(size(t))';
r(1:100) = 4;
r(100:250) = -5;
r(250:end) = 3;

%% DO GENETIC
for gen=1:numgen
    Fit=fitness_pid(Pop, Gs, r, t, true, 0.5);

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
    fprintf('Generation %d completed.\n', gen);
end

%% Save the best chromo
save(FILENAME_TRAIN + "/bestchrom_pid.mat", 'bestchrom', 'best', "evolution");

%% Save and Draw Best
close all;
if ~exist(FILENAME_TRAIN + "/bestchrom_pid.mat", 'file')
    if ~exist('rezim', 'var') || ~exist('bestchrom', 'var') || ~exist('evolution', 'var')
        error('Variables "layers", "rezim", "bestchrom", and "evolution" must be defined in the workspace.');
    end
end

load(FILENAME_TRAIN + "/bestchrom_pid.mat");
x0 = zeros(numel(Gs.den)-1, 1);

[t,y,dy,w,e,de]=sim_pid_VIR25(...
    bestchrom(1), ...
    bestchrom(2), ...
    bestchrom(3), ...
    Gs, r, t, x0);

figure(1);
stairs(evolution);
title('Evolution of PID Controller');
xlabel('Generation');
ylabel('Fitness');
grid on;
saveas(gcf, FILENAME_TRAIN + "/evolution_pid.svg");

figure(2);
hold on;
stairs(t, y, 'b');
stairs(t, w, 'r--');
title('Output and Reference Signal');
xlabel('Time [s]');
ylabel('Amplitude');
legend('Output', 'Reference');
grid on;
hold off;
saveas(gcf, FILENAME_TRAIN + "/output_pid_best_train.svg");

%% Test Trained

if ~exist('bestchrom', 'var') || ~exist('evolution', 'var')
    error('Variables "layers", "rezim", "bestchrom", and "evolution" must be defined in the workspace.');
end

% bestchrom = [1, 0.2, 10];
r_bestchrom = round(bestchrom, 3);
Gs = Gs21;
t = 0:dt:20;
r = ones(size(t))';
r(1:100) = 5;
r(100:200) = 0;
r(200:300) = -7;
r(300:end) = 10;

x0 = zeros(numel(Gs.den)-1, 1);
umax = 100;
umin = -100;
Imax = 85; % Anti-windup upper saturation
Imin = -85; % Anti-windup lower saturation


[t,y,d1y,w,e,de,u,ureal]=sim_pid_VIR25(...
    r_bestchrom(1), ...
    r_bestchrom(2), ...
    r_bestchrom(3), ...
    Gs, r, t, ...
    x0, ...
    umin, ...
    umax, ...
    Imin, ...
    Imax); % test rezim

figure(3);
hold on;
stairs(t, y);
stairs(t, w, 'k--');
stairs(t, ureal, 'r');
title('Output and Reference Signal');
subtitle("[P, I, D] = " + mat2str(r_bestchrom))
xlabel('Time [s]');
ylabel('Amplitude');
legend('y', 'ref', 'u');
grid on;
saveas(gcf, FILENAME_TRAIN + "/output_pid_best_test.svg");
hold off;

fprintf("chrom = " + mat2str(r_bestchrom) + ";\n");