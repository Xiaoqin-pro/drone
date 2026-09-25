function RunAdditionPairedDiagnostic
%RUNADDITIONPAIREDDIAGNOSTIC 单事件新增订单的配对诊断实验
%   当前默认每个M级别运行2个实例；确认后可改为10。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
manifest = readtable(fullfile(root,'results','addition_instances', ...
    'addition_manifest.csv'),'TextType','string');
levels = unique(string(manifest.level),'stable');
strategies = {'Restart','WarmStart','Repair'};
budgets = [500 1000 1500 2500];
nInstancesPerLevel = 2;
cfg.population = 30;
cfg.inertia = 0.90;
cfg.inertiaDamp = 0.995;
cfg.c1 = 1.7;
cfg.c2 = 1.7;
cfg.referenceMaxTime = 10;
cfg.outputDir = fullfile(root,'results');
rows = cell(0,1);

for levelIndex = 1:numel(levels)
    levelRows = manifest(string(manifest.level)==levels(levelIndex),:);
    levelRows = levelRows(1:min(nInstancesPerLevel,height(levelRows)),:);
    for r = 1:height(levelRows)
        instance = load(fullfile(root,'results','addition_instances', ...
            levelRows.file_name(r)));
        instance = instance.instance;
        baseModel = CreateModel();
        baseModel.windows(:,2) = baseModel.windows(:,2)+300;
        baseModel.cfg.silent = true;
        activeInitial = baseModel.customerIDs(:)';
        baseCache = BuildLegCache(baseModel,activeInitial, ...
            baseModel.homePosition);
        rng(860000,'twister');
        [initialSolution,~,~] = RoutingPSO(baseModel,baseCache, ...
            100,40,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

        model = instance.model;
        model.startTime = instance.state.time;
        model.depot = instance.state.position;
        activeIDs = instance.activeIDs;
        cache = instance.cache;
        event = instance.event;
        oracleStatus = "KNOWN_FEASIBLE_WITNESS";
        referenceCost = instance.referenceCost;

        for strategyIndex = 1:numel(strategies)
            for budgetIndex = 1:numel(budgets)
                strategy = strategies{strategyIndex};
                budget = budgets(budgetIndex);
                algorithmSeed = 7600+100*r+budget;
                [solution,metrics] = RunStrategyFromState( ...
                    strategy,initialSolution,model,cache,activeIDs, ...
                    event,budget,cfg,algorithmSeed);
                gap = (solution.Cost-referenceCost)/ ...
                    max(abs(referenceCost),eps);
                rows{end+1,1} = {levels(levelIndex),r, ...
                    instance.scenarioSeed,algorithmSeed,budget, ...
                    string(strategy),event.time,numel(event.customerIDs), ...
                    oracleStatus,referenceCost,metrics.warmStartFE, ...
                    metrics.routingFE,metrics.totalFE,metrics.responseTime, ...
                    solution.Cost,solution.Detail.distance, ...
                    solution.Detail.totalLate,solution.Detail.totalObstacleViolation, ...
                    solution.Detail.isFeasible,gap, ...
                    ComputeRouteDisruption(initialSolution.Detail.routeIDs, ...
                    solution.Detail.routeIDs)}; %#ok<AGROW>
            end
        end
        fprintf('%s addition instance %d/%d completed.\n', ...
            levels(levelIndex),r,nInstancesPerLevel);
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','algorithm_seed','budget', ...
    'strategy','event_time','event_size','oracle_status', ...
    'reference_cost','warm_start_fe','routing_fe','total_fe', ...
    'response_time','fitness','distance','total_late', ...
    'obstacle_violation','is_feasible','gap_to_reference', ...
    'route_disruption'});
writetable(summary,fullfile(cfg.outputDir, ...
    'addition_paired_summary.csv'));
save(fullfile(cfg.outputDir,'addition_paired_result.mat'), ...
    'summary','levels','strategies','budgets','cfg');
fprintf('\nSaved addition paired diagnostic results.\n');
end

function [solution,metrics] = RunStrategyFromState(strategy, ...
    previousSolution,model,cache,activeIDs,event,budget,cfg,algorithmSeed)
metrics.warmStartFE = 0;
metrics.warmStartTime = 0;
if strcmpi(strategy,'Repair')
    timer = tic;
    [solution,repairInfo] = LocalRepair(previousSolution,activeIDs, ...
        model,cache,event.type,event.customerIDs);
    metrics.routingTime = toc(timer);
    metrics.routingFE = repairInfo.functionEvaluations;
else
    if strcmpi(strategy,'WarmStart')
        rng(algorithmSeed,'twister');
        timer = tic;
        [initialPositions,warmInfo] = BuildWarmStartPopulation( ...
            previousSolution,activeIDs,model,cache,cfg.population);
        metrics.warmStartTime = toc(timer);
        metrics.warmStartFE = warmInfo.evaluations;
    else
        initialPositions = [];
    end
    availableFE = max(0,budget-metrics.warmStartFE);
    maxIt = max(0,floor(availableFE/cfg.population)-1);
    rng(algorithmSeed,'twister');
    timer = tic;
    [solution,~,stats] = RoutingPSO(model,cache,maxIt,cfg.population, ...
        cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,initialPositions);
    metrics.routingTime = toc(timer);
    metrics.routingFE = stats.functionEvaluations;
end
metrics.totalFE = metrics.warmStartFE+metrics.routingFE;
metrics.responseTime = metrics.routingTime+metrics.warmStartTime;
end


