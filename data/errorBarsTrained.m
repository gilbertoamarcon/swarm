avgs = [];
bests = [];
mins = [];
plots = [];
results = [];
linestyles = {'-o','-s','--','.-','-^','-o','-s','--','.-','-^'}
R = 20;      % # Robots
L = 16;      % # Leaders
for L=0:2:18
    avgs = [];
    bests = [];
    mins = [];
    E = 50;     % # Epochs
    S = 99;     % # Statistical Runs
    files = {};
    for i=0:S
        files = [files, [num2str(i) '_TRAINING_DATA_R' num2str(R) '_L' num2str(L) '_E' num2str(E) '.txt']];
    end

    % Get averages, bests, worsts
    for i=1:length(files)
%         files{i}
        errors = csvread(files{i});
        errors = errors(:,1:size(errors,2)-1)';
        avgs = [avgs mean(errors)'];
    %     bests = [bests max(errors)'];
    %     mins = [mins min(errors)'];
    end
    avgst = avgs';
    avgsLast10 = avgst(:,end-9:end);
    results = [results mean(avgsLast10')';]
%     hold on
%     h(L/2+1) = errorBars(avgs, linestyles{L/2+1});
    
    % errorBars(bests);
    % errorBars(mins);

    % title('Learning Curves for Best, Average, Worst MLPs');
    % xlabel('Time')
    % ylabel('Distance to Goal')
%     legend('Average Mlp', 'Best Mlp', 'Worst Mlp');
end

boxplot(results, 'BoxStyle', 'filled', 'Colors', [60, 80, 115]/255, 'OutlierSize', 4)
h = findobj(gcf,'tag','Outliers');
for iH = 1:length(h)
    h(iH).MarkerEdgeColor = [120, 130, 180]/255;
end

title('Distance to goal by ratio of leaders');
ylabel('Accumulated distance to Goal')
xlabel('Leader/Swarm Ratio')
set(gca, 'xtick', [1:10],'xticklabel', [0:0.1:0.9])
