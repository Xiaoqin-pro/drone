function AnalyzeAdditionResults
%ANALYZEADDITIONRESULTS 分析30个新增订单诊断实例
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
summary = readtable(fullfile(resultDir,'addition_paired_summary.csv'), ...
    'TextType','string');

[G,level,strategy,budget] = findgroups( ...
    summary.level,summary.strategy,summary.budget);
means = table(level,strategy,budget, ...
    splitapply(@mean,summary.total_fe,G), ...
    splitapply(@mean,summary.fitness,G), ...
    splitapply(@mean,summary.gap_to_reference,G), ...
    splitapply(@mean,double(summary.is_feasible),G), ...
    splitapply(@mean,summary.response_time,G), ...
    'VariableNames',{'level','strategy','budget','mean_total_fe', ...
    'mean_fitness','mean_gap','feasibility_rate','mean_response_time'});
writetable(means,fullfile(resultDir,'addition_paired_means.csv'));

f = figure('Visible','off','Color','w');
levels = unique(string(summary.level),'stable');
strategies = unique(string(summary.strategy),'stable');
budgets = unique(summary.budget)';
colors = lines(numel(strategies));
for l = 1:numel(levels)
    subplot(1,numel(levels),l);
    hold on;
    for s = 1:numel(strategies)
        rows = string(summary.level)==levels(l) ...
            & string(summary.strategy)==strategies(s);
        local = summary(rows,:);
        [x,order] = sort(local.total_fe);
        y = local.gap_to_reference(order);
        plot(x,y,'-o','Color',colors(s,:), ...
            'LineWidth',1.7,'DisplayName',strategies(s));
    end
    title(levels(l));
    xlabel('Actual FE');
    ylabel('Gap to reference');
    grid on;
    legend('Location','best');
end
exportgraphics(f,fullfile(resultDir,'addition_gap_vs_fe.png'), ...
    'Resolution',180);

fprintf('Saved addition analysis to %s\n',resultDir);
end
