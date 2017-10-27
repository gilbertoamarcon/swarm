means = [];
errs = [];

results = {'MLP_12R_8L_M.txt', ...
           'MLP_BASE_12R_8L_M.txt', ...
           'MLP_12R_8L_T.txt', ...
           'MLP_BASE_12R_8L_T.txt', ...
           'MLP_12R_8L_V.txt', ...
           'MLP_BASE_12R_8L_V.txt'};
for i=results
    errors = csvread(cell2mat(i));
    errors = errors(:,1:size(errors,2)-1);
    m = mean(errors');
    s = std(errors');

% %   Results on Best NN
%     [bestMlpMean, bestMlpIndex] = max(m);
%     bestMlpStd = s(bestMlpIndex);
%     means = [means, bestMlpMean];
%     errs = [errs, bestMlpStd/sqrt(size(errors',2))];
    
%   Results on Average of all NNs
    averageMlpMean = mean(m);
    averageMlpStd = std(reshape(errors,1,[]));
    means = [means, averageMlpMean];
    errs = [errs, averageMlpStd/sqrt(size(errors',2))];
end

means = vec2mat(means, 2) %#ok
errs = vec2mat(errs, 2);

hold on
bar_handle = bar(means);
set(bar_handle(1), 'FaceColor', [72, 86, 122]/255);
set(bar_handle(2), 'FaceColor', [204, 194, 146]/255);
w = 0.14;
errorbar((1:3)-w, means(1:3), errs(1:3), '.k')
errorbar((1:3)+w, means(4:6), errs(4:6), '.k')

title('Distance to Goal by Communication Model and Learning (Average NN)');
legend('Learning Leaders', 'Static Leaders');
ylabel('Distance to Goal')
names = {'Metric'; 'Topological'; 'Visual'};
set(gca,'xtick',[1:3],'xticklabel',names)