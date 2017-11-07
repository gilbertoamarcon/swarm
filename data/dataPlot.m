means = [];
errs = [];

results = {'TRAINING_DATA_R6_L4_E100.txt', ...
           'TRAINING_DATA_R10_L4_E100.txt', ...
           'TRAINING_DATA_R14_L4_E100.txt', ...
           'TRAINING_DATA_R18_L4_E100.txt', ...
           'TRAINING_DATA_R22_L4_E100.txt', ...
           'TRAINING_DATA_R26_L4_E100.txt', ...
           'TRAINING_DATA_R30_L4_E100.txt'};
% linestyle = {'.', '*', 'd', '*', 'd', '*'};
hold on
for i=1:length(results)
    errors = csvread(results{i});
    errors = errors(1:100,1:size(errors,2)-1);
    m = mean(errors');
    e = std(errors')/sqrt(size(errors,2));
    plot(m)
end

title('Distance to Goal by Communication Model (Learning Process)');
ylabel('Distance to Goal')
% legend('Metric', 'Topological', 'Visual');



% 
% f = figure();
% plot(min(errors'), 'Color', [0.2 0.6 0.5]);
% saveas(f,'plot.svg');


% plot(mean(errors'), 'Color', [0.2 0.6 0.5])
% hold on
% h = errorbar(mean(errors'), std(errors') * sqrt(size(errors,2)-1), 'k.'); % 0.6198 = 1.96 * sqrt(10), where 10 is sample size

% exit;