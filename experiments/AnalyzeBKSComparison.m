function AnalyzeBKSComparison
%ANALYZEBKSCOMPARISON 用离线BKS候选重新计算质量Gap
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
bksFile = fullfile(resultDir,'offline_bks_summary_updated.csv');
if ~isfile(bksFile)
    bksFile = fullfile(resultDir,'offline_bks_summary.csv');
end
bks = readtable(bksFile,'TextType','string');

files = {fullfile(resultDir,'addition_paired_summary.csv'), ...
    fullfile(resultDir,'stress_addition_summary.csv')};
outputs = {fullfile(resultDir,'addition_paired_bks_gap.csv'), ...
    fullfile(resultDir,'stress_addition_bks_gap.csv')};
datasets = ["addition_instances","addition_stress_instances"];
means = cell(0,1);

for f = 1:numel(files)
    if ~isfile(files{f})
        continue;
    end
    data = readtable(files{f},'TextType','string');
    data.gap_to_bks = nan(height(data),1);
    data.bks_cost = nan(height(data),1);
    for r = 1:height(data)
        if f==1
            bksRows = bks.dataset==datasets(f) & bks.level==data.level(r) ...
                & bks.instance==data.instance(r);
        else
            bksRows = bks.dataset==datasets(f) & bks.level==data.stress_level(r) ...
                & bks.instance==data.instance(r);
        end
        if any(bksRows)
            bksCost = bks.bks_cost(find(bksRows,1));
            data.bks_cost(r) = bksCost;
            data.gap_to_bks(r) = ...
                (data.fitness(r)-bksCost)/max(abs(bksCost),eps);
        end
    end
    writetable(data,outputs{f});
    if f==1
        groupLevel=data.level;
    else
        groupLevel=data.stress_level;
    end
    [G,level,strategy,budget] = findgroups(groupLevel,data.strategy,data.budget);
    means = [means; {table(level,strategy,budget, ...
        splitapply(@mean,data.gap_to_bks,G), ...
        splitapply(@mean,data.total_fe,G), ...
        splitapply(@mean,double(data.is_feasible),G), ...
        'VariableNames',{'level','strategy','budget', ...
        'mean_gap_to_bks','mean_total_fe','feasibility_rate'})}]; %#ok<AGROW>
end

for k=1:numel(means)
    if k==1
        combined=means{k};
    else
        combined=[combined;means{k}]; %#ok<AGROW>
    end
end
writetable(combined,fullfile(resultDir,'bks_gap_means.csv'));
disp(combined);
end
