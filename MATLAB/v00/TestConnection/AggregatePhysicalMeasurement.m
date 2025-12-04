DIR = "dataPhys";

allMatFiles = dir(DIR);

% Only select the CSV files

includeFilter = ".csv";

csvFiles = allMatFiles(contains({allMatFiles.name}, includeFilter));

% Initialize a cell array to store the full paths of the CSV files
csvFilePaths = fullfile(DIR, {csvFiles.name});
% Read and process each CSV file
data = cell(1, numel(csvFilePaths));


for i = 1:length(csvFilePaths)
    data{i} = readtable(csvFilePaths{i}, "Delimiter", ",", "VariableNamingRule","preserve");
end

% Get minimal length per data
minLength = min(cellfun(@(x) height(x), data));

smask = 1:minLength;

ncols = numel(data) + 1;

ts = zeros(minLength, ncols);
tps = zeros(minLength, ncols);
ys = zeros(minLength, ncols);

for i = 1:length(csvFilePaths)
    ts(:, i) = data{i}.t(smask);
    tps(:, i) = data{i}.tp(smask);
    ys(:, i) = data{i}.y(smask);
    ys(:, i) = ys(:, i) - mean(ys(end-50:end, i));
end

ts(:, end) = mean(ts(:, 1:end-1), 2);
tps(:, end) = mean(tps(:, 1:end-1), 2);
ys(:, end) = mean(ys(:, 1:end-1), 2);

%% Plot all the outputs within the same graph
N = size(tps, 2);

figure(123); clf;
hold on; % Retain current plot when adding new data
for i = 1:N
    if i == N
        plot(tps(:, i), ys(:, i), "--k", "LineWidth", 1.5, 'DisplayName', "ym"); % Assuming Var1 and Var2 are the columns to plot
        % plot(ts(:, i), ys(:, i), "-k", "LineWidth", 1.5, 'DisplayName', "ym"); % Assuming Var1 and Var2 are the columns to plot
    else
        plot(tps(:, i), ys(:, i), 'DisplayName', "y" + num2str(i)); % Assuming Var1 and Var2 are the columns to plot
        % plot(ts(:, i), ys(:, i), "--r", "LineWidth", 1.5, 'DisplayName', "ym"); % Assuming Var1 and Var2 are the columns to plot
    end
end
hold off; % Release the plot hold
grid minor;
grid on;
xlabel('t [s]');
ylabel("$\varphi$ [deg]", "Interpreter", "latex");
title("Physical model constants identification");
legend show;