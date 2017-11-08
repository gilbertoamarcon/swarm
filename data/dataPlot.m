avgs = [];
bests = [];
mins = [];

R = 5;     % # Robots
L = 3;      % # Leaders
E = 50;    % # Epochs
S = 99;      % # Statistical Runs
files = {};
for i=1:S
    files = [files, [num2str(i) '_TRAINING_DATA_R' num2str(R) '_L' num2str(L) '_E' num2str(E) '.txt']];
end

% Get averages, bests, worsts
for i=1:length(files)
    files{i}
    errors = csvread(files{i});
    errors = errors(:,1:size(errors,2)-1)';
    avgs = [avgs mean(errors)'];
%     bests = [bests max(errors)'];
%     mins = [mins min(errors)'];
end

hold on
errorBars(avgs);
errorBars(bests);
errorBars(mins);

title('Learning Curves for Best, Average, Worst MLPs');
xlabel('Time')
ylabel('Distance to Goal')
legend('Average Mlp', 'Best Mlp', 'Worst Mlp');

function errorBars(data)
% Plot errorbar plot
% Rows of data are performances over time, columns are statistical runs
m = mean(data');                         % Means 
e = std(data')/sqrt(size(data',2));       % Std Error
plot(m)
% h = errorbar(e, m);
end
