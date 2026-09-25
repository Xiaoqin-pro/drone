function MergeKnownSolutionsIntoBKS
%MERGEKNOWNSOLUTIONSINTOBKS 更新best-known候选，纳入所有已运行策略结果
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
bks = readtable(fullfile(resultDir,'offline_bks_summary.csv'),'TextType','string');
files = {fullfile(resultDir,'addition_paired_summary.csv'), ...
    fullfile(resultDir,'stress_addition_summary.csv')};
datasets = ["addition_instances","addition_stress_instances"];
methods = bks.bks_method;
for f=1:numel(files)
    if ~isfile(files{f}), continue; end
    data=readtable(files{f},'TextType','string');
    for r=1:height(data)
        if f==1, level=data.level(r); else, level=data.stress_level(r); end
        rows=bks.dataset==datasets(f) & bks.level==level ...
            & bks.instance==data.instance(r);
        if any(rows)
            idx=find(rows,1);
            if data.fitness(r)<bks.bks_cost(idx)
                bks.bks_cost(idx)=data.fitness(r);
                bks.bks_method(idx)="observed_"+data.strategy(r);
                bks.bks_seed(idx)=data.scenario_seed(r);
            end
        end
    end
end
writetable(bks,fullfile(resultDir,'offline_bks_summary_updated.csv'));
end
