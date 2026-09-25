function AnalyzeDiagnosticResults
%ANALYZEDIAGNOSTICRESULTS 输出Benchmark Protocol v1诊断汇总
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
summary = readtable(fullfile(resultDir,'paired_diagnostic_summary.csv'));

% 同一实例、预算、事件内的经验相对差距：相对于三种策略的最好Fitness。
summary.relative_gap_to_best = zeros(height(summary),1);
groupKeys = string(summary.level)+"_"+string(summary.instance)+"_" ...
    +string(summary.budget)+"_"+string(summary.event_index);
uniqueKeys = unique(groupKeys,'stable');
for k = 1:numel(uniqueKeys)
    rows = groupKeys==uniqueKeys(k);
    bestValue = min(summary.final_fitness(rows));
    summary.relative_gap_to_best(rows) = ...
        (summary.final_fitness(rows)-bestValue)/max(abs(bestValue),eps);
end

writetable(summary,fullfile(resultDir,'paired_diagnostic_with_gap.csv'));

[G,level,strategy,budget,eventIndex] = findgroups( ...
    summary.level,summary.strategy,summary.budget,summary.event_index);
means = table(level,strategy,budget,eventIndex, ...
    splitapply(@mean,summary.response_time,G), ...
    splitapply(@mean,summary.total_fe,G), ...
    splitapply(@mean,summary.final_fitness,G), ...
    splitapply(@mean,summary.total_late,G), ...
    splitapply(@mean,summary.route_disruption,G), ...
    splitapply(@mean,double(summary.is_feasible),G), ...
    splitapply(@mean,summary.relative_gap_to_best,G), ...
    'VariableNames',{'level','strategy','budget','event_index', ...
    'mean_response_time','mean_total_fe','mean_fitness','mean_late', ...
    'mean_route_disruption','feasibility_rate','mean_relative_gap'});
writetable(means,fullfile(resultDir,'paired_diagnostic_means.csv'));

% 绘制可行率和相对差距
levels = unique(string(summary.level),'stable');
strategies = unique(string(summary.strategy),'stable');
budgets = unique(summary.budget)';

f = figure('Visible','off','Color','w');
tiledlayout(1,2);
nexttile;
hold on;
for s = 1:numel(strategies)
    values = zeros(numel(budgets),1);
    for b = 1:numel(budgets)
        rows = string(summary.strategy)==strategies(s) & summary.budget==budgets(b);
        values(b) = mean(double(summary.is_feasible(rows)));
    end
    plot(budgets,values,'-o','LineWidth',1.8,'DisplayName',strategies(s));
end
grid on; ylim([0 1.05]);
xlabel('Total FE budget'); ylabel('Feasibility rate');
title('Feasibility rate by total FE');
legend('Location','best');

nexttile;
hold on;
for s = 1:numel(strategies)
    values = zeros(numel(budgets),1);
    for b = 1:numel(budgets)
        rows = string(summary.strategy)==strategies(s) & summary.budget==budgets(b);
        values(b) = mean(summary.relative_gap_to_best(rows));
    end
    plot(budgets,values,'-o','LineWidth',1.8,'DisplayName',strategies(s));
end
grid on;
xlabel('Total FE budget'); ylabel('Relative gap to best in paired condition');
title('Quality gap under equal FE budget');
legend('Location','best');
exportgraphics(f,fullfile(resultDir,'paired_diagnostic_curves.png'), ...
    'Resolution',180);

fprintf('Saved diagnostic analysis to %s\n',resultDir);
end



