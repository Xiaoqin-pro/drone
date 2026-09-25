function BuildOfflineBKS
%BUILDOFFLINEBKS 为已有新增订单实例生成离线Best-known candidate
%   witness只证明可行；BKS候选用于质量Gap，不声称全局最优。

clc
clear
close all

scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
datasets = {'addition_instances','addition_stress_instances'};
nStarts = 3;
totalBudget = 3000;
population = 60;
rows = cell(0,1);

for d = 1:numel(datasets)
    dataDir = fullfile(resultDir,datasets{d});
    manifestPath = dir(fullfile(dataDir,'*manifest.csv'));
    if isempty(manifestPath)
        continue;
    end
    manifest = readtable(fullfile(dataDir,manifestPath(1).name), ...
        'TextType','string');
    fileColumn = string(manifest.file_name);
    if ismember('level',manifest.Properties.VariableNames)
        levelColumn = string(manifest.level);
    else
        levelColumn = string(manifest.stress_level);
    end
    for r = 1:height(manifest)
        loaded = load(fullfile(dataDir,fileColumn(r)));
        instance = loaded.instance;
        model = instance.model;
        model.cfg.silent = true;
        cache = instance.cache;
        activeIDs = instance.activeIDs;
        bestCost = instance.referenceCost;
        bestSolution = struct();
        bestSolution.Cost = instance.referenceCost;
        bestSolution.Detail.routeIDs = instance.referenceRoute;
        bestMethod = "witness";
        bestSeed = NaN;
        totalFE = 0;

        maxIt = max(0,floor(totalBudget/population)-1);
        for s = 1:nStarts
            seed = 910000+d*10000+r*100+s;
            rng(seed,'twister');
            [solution,~,stats] = RoutingPSO(model,cache,maxIt, ...
                population,0.90,0.995,1.7,1.7,[]);
            totalFE = totalFE+stats.functionEvaluations;
            if solution.Detail.isFeasible && solution.Cost<bestCost
                bestCost = solution.Cost;
                bestSolution = solution;
                bestMethod = "restart";
                bestSeed = seed;
            end
        end

        rows{end+1,1} = {string(datasets{d}), ...
            levelColumn(r),manifest.instance(r), ...
            manifest.scenario_seed(r),bestCost,bestMethod,bestSeed, ...
            totalFE,instance.referenceCost}; %#ok<AGROW>
        fprintf('%s %s %d: witness=%.2f, BKS=%.2f, method=%s\n', ...
            datasets{d},levelColumn(r),manifest.instance(r), ...
            instance.referenceCost,bestCost,bestMethod);
    end
end

bks = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','bks_cost','bks_method', ...
    'bks_seed','bks_search_fe','witness_cost'});
writetable(bks,fullfile(resultDir,'offline_bks_summary.csv'));
save(fullfile(resultDir,'offline_bks_result.mat'), ...
    'bks','nStarts','totalBudget','population');
end
