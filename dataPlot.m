errors = csvread('MLP_errors.txt');
errors = errors(:,1:size(errors,2)-1);
f = figure();
plot(min(errors'), 'Color', [0.2 0.6 0.5]);
saveas(f,'plot.svg');

% plot(mean(errors'), 'Color', [0.2 0.6 0.5])
% hold on
% h = errorbar(mean(errors'), std(errors') * sqrt(size(errors,2)-1), 'k.'); % 0.6198 = 1.96 * sqrt(10), where 10 is sample size

exit;