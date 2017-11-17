results = [];

for folder = {'NoCentroid', 'Obstacles10', 'Obstacles20', 'SwarmPull25'}
    cd(cell2mat(folder));
    avgs = [];
    bests = [];
    mins = [];
    R = 20;      % # Robots
    L = 4;      % # Leaders
    E = 50;     % # Epochs
    S = 10;     % # Statistical Runs
    files = {};
    for i=0:S-1
        [num2str(i) '_TRAINING_DATA_R' num2str(R) '_L' num2str(L) '_E' num2str(E) '.txt']
        files = [files, [num2str(i) '_TRAINING_DATA_R' num2str(R) '_L' num2str(L) '_E' num2str(E) '.txt']];
    end

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
    cd ..
end


% subplot(1,3,3)
boxplot(results, 'BoxStyle', 'filled', 'Colors', [60, 80, 115]/255, 'OutlierSize', 4)
h = findobj(gcf,'tag','Outliers');
for iH = 1:length(h)
    h(iH).MarkerEdgeColor = [120, 130, 180]/255;
end

title('15 Leaders');
ylabel('Accumulated distance to Goal')
xlabel('Control Strategy')
set(gca, 'xtick', [1:10],'xticklabel', {'NoCentroid', '10Obstacles', '20Obstacles', 'SwarmPull25'});
