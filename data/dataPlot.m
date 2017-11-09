avgs = [];
bests = [];
mins = [];
plots = [];
linestyles = {'-o','-s','--','.-','-^'}
R = 20;      % # Robots
L = 16;      % # Leaders
for L=0:4:16
    E = 50;     % # Epochs
    S = 50;     % # Statistical Runs
    files = {};
    for i=0:S
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
    h(L/4+1) = errorBars(avgs, linestyles{L/4+1});
    
    % errorBars(bests);
    % errorBars(mins);

    % title('Learning Curves for Best, Average, Worst MLPs');
    % xlabel('Time')
    % ylabel('Distance to Goal')
%     legend('Average Mlp', 'Best Mlp', 'Worst Mlp');
end

title('Learning Curves, averaged over population');
xlabel('Epochs')
ylabel('Distance to Goal')
l = legend(h, '0.0', '0.2', '0.4', '0.6', '0.8');
title(l,'Leader-Swarm Ratio')

function h = errorBars(data, style)
    % Plot errorbar plot
    % Rows of data are performances over time, columns are statistical runs
    m = mean(data');                         % Means 
    
    e = std(data')/sqrt(50)/2;       % Std Error, 95% Confidence
    h =plot(m, style);
    eri = 5; % Errorbar interval
    errorbar([1:eri:length(m)], m(1:eri:end), e(1:eri:end), '.', 'Color', [0.6 0.6 0.6]);
end
