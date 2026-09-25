function PlotMechanismV2
%PLOTMECHANISMV2 绘制Strict Anytime与事件影响机制图
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
t=readtable(fullfile(resultDir,'strict_mechanism_table.csv'),'TextType','string');

% 1. 三策略的中位Gap随FE变化
f=figure('Visible','off','Color','w');
strategies={'Repair','WarmStart','Restart'};
colors=[0.10 0.55 0.20;0.10 0.35 0.80;0.80 0.20 0.10];
levels=unique(t.level,'stable');
for l=1:numel(levels)
    subplot(1,numel(levels),l); hold on;
    for s=1:numel(strategies)
        rows=string(t.level)==levels(l) & string(t.best_strategy)~="";
        fieldName = 'repair_gap';
        if s==2, fieldName='warm_gap'; elseif s==3, fieldName='restart_gap'; end
        rows=rows & isfinite(t.(fieldName));
        if any(rows)
            [x,~,g]=unique(t.checkpoint_fe(rows));
            values = t.(fieldName);
            y=splitapply(@median,values(rows),g);
            plot(x,y,'-o','Color',colors(s,:),'LineWidth',1.6, ...
                'DisplayName',strategies{s});
        end
    end
    grid on; title(levels(l)); xlabel('FE'); ylabel('Median BKS gap');
    legend('Location','best');
end
exportgraphics(f,fullfile(resultDir,'mechanism_anytime_gap.png'),'Resolution',180);

% 2-4. 特征与策略收益
f=figure('Visible','off','Color','w');
tiledlayout(1,3);
nexttile; scatter(t.insertion_ratio,t.warm_gain_vs_repair,28, ...
    t.checkpoint_fe,'filled'); colorbar; grid on;
xlabel('Insertion ratio'); ylabel('Warm gain vs Repair'); title('Spatial insertion impact');
nexttile; scatter(t.slack_loss,t.warm_gain_vs_repair,28, ...
    t.checkpoint_fe,'filled'); colorbar; grid on;
xlabel('Slack loss'); ylabel('Warm gain vs Repair'); title('Time-window propagation');
nexttile; scatter(t.slack_loss,t.restart_gain_vs_warm,28, ...
    t.checkpoint_fe,'filled'); colorbar; grid on;
xlabel('Slack loss'); ylabel('Restart gain vs Warm'); title('History validity');
exportgraphics(f,fullfile(resultDir,'mechanism_feature_gains.png'),'Resolution',180);

% 5. 插入影响×Slack损失，颜色表示当前最佳策略
f=figure('Visible','off','Color','w'); hold on; grid on;
labels=unique(t.best_strategy,'stable');
colors2=lines(max(numel(labels),1));
for k=1:numel(labels)
    rows=t.best_strategy==labels(k);
    scatter(t.insertion_ratio(rows),t.slack_loss(rows),35, ...
        colors2(k,:),'filled','DisplayName',labels(k));
end
xlabel('Insertion ratio'); ylabel('Slack loss');
title('Event-impact map and best strategy');
legend('Location','best');
exportgraphics(f,fullfile(resultDir,'mechanism_impact_map.png'),'Resolution',180);
end
