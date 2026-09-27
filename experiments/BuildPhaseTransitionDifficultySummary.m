function results = BuildPhaseTransitionDifficultySummary
%BUILDPHASETRANSITIONDIFFICULTYSUMMARY 从现有pilot结果筛选boundary-active实例
%   不重新运行算法；优先推荐n=21、统一budget下有部分seed跨过boundary的realization。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
inputFile=fullfile(root,'results','phase_transition_budget_screen_summary.csv');
t=readtable(inputFile,'TextType','string');
keys=unique(t(:,{'benchmark_nodes','beta','realization','budget_factor'}),'rows');
rows=cell(0,1);
for i=1:height(keys)
    m=t.benchmark_nodes==keys.benchmark_nodes(i) & ...
        abs(t.beta-keys.beta(i))<1e-10 & t.realization==keys.realization(i) & ...
        t.budget_factor==keys.budget_factor(i);
    x=t(m,:); first=double(x.first_feasible_fe); first=first(isfinite(first));
    rate=mean(logical(x.is_feasible));
    rows{end+1,1}={keys.benchmark_nodes(i),keys.beta(i),keys.realization(i), ...
        keys.budget_factor(i),height(x),rate,MedianOrNaN(first), ...
        MedianOrNaN(double(x.total_late)), ...
        MedianOrNaN(double(x.first_feasible_fe)/max(double(x.budget_fe(1)),eps)), ...
        rate>=1/3 && rate<=2/3}; %#ok<AGROW>
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'benchmark_nodes','beta','realization','budget_factor','seed_count', ...
    'feasible_rate','median_first_feasible_fe','median_total_late', ...
    'median_first_feasible_fraction','boundary_active'});

% 优先统一200n；若候选不足4个，则使用统一500n。
selectedBudget=200;
mask=summary.benchmark_nodes==21 & summary.budget_factor==selectedBudget & summary.boundary_active;
if sum(mask)<4
    selectedBudget=500;
    mask=summary.benchmark_nodes==21 & summary.budget_factor==selectedBudget & summary.boundary_active;
end
selected=summary(mask,:);
if height(selected)>0
    distance=abs(selected.feasible_rate-0.5);
    [~,order]=sort(distance,'ascend'); selected=selected(order,:);
    selected=selected(1:min(6,height(selected)),:);
end
results.selectedBudget=selectedBudget;
results.summary=summary; results.selected=selected;
writetable(summary,fullfile(root,'results','phase_transition_difficulty_summary.csv'));
writetable(selected,fullfile(root,'results','phase_transition_selected_boundary.csv'));
save(fullfile(root,'results','phase_transition_difficulty_summary.mat'),'results');
fprintf('Phase-transition difficulty summary completed: %d groups, %d selected candidates.\n', ...
    height(summary),height(selected));
end

function value=MedianOrNaN(x)
x=x(isfinite(x)); if isempty(x), value=NaN; else, value=median(x); end
end
