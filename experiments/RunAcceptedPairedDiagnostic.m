function RunAcceptedPairedDiagnostic
%RUNACCEPTEDPAIREDDIAGNOSTIC 在冻结实例上进行无信息泄漏的单事件配对实验
%   参考路线只用于离线Gap评价，不作为任何策略的输入。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
manifest = readtable(fullfile(root,'results', ...
    'accepted_benchmark_manifest.csv'),'TextType','string');
strategies = {'Restart','WarmStart','Repair'};
budgets = [500 1000 1500 2500];
cfg.population = 30;
cfg.inertia = 0.90;
cfg.inertiaDamp = 0.995;
cfg.c1 = 1.7;
cfg.c2 = 1.7;
cfg.referenceMaxTime = 10;
cfg.outputDir = fullfile(root,'results');
rows = cell(0,1);

for instanceRow = 1:height(manifest)
    level = char(manifest.level(instanceRow));
    instanceID = manifest.instance(instanceRow);
    scenarioSeed = manifest.scenario_seed(instanceRow);
    loaded = load(fullfile(root,'results',manifest.file_name(instanceRow)));
    instance = loaded.instance;

    % 固定事件前模型和初始路线；每个事件都从同一状态单独开始。
    baseModel = CreateModel();
    baseModel.cfg.silent = true;
    activeInitial = baseModel.customerIDs(:)';
    initialCache = BuildLegCache(baseModel,activeInitial, ...
        baseModel.homePosition);
    rng(880000+scenarioSeed,'twister');
    [initialSolution,~,~] = RoutingPSO(baseModel,initialCache,50,30, ...
        cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

    for e = 1:numel(instance.events)
        % 重要：不使用instance.audit中的参考路线作为算法输入。
        event = instance.events(e);
        state = ExecuteUntilEvent(initialSolution,baseModel,event.time);
        activeIDs = setdiff(activeInitial,state.completedIDs,'stable');
        model = ApplyEvent(baseModel,event);
        if strcmpi(event.type,'cancel')
            activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
        else
            activeIDs = [activeIDs,event.customerIDs]; %#ok<AGROW>
        end
        model.startTime = event.time;
        model.depot = state.position;
        [cache,cacheStats] = UpdateLegCache(model,initialCache, ...
            activeIDs,state.position);

        % 参考路线只用于Gap，不进入Restart/Warm/Repair。
        oracle = ExactTWFeasibilityOracle(model,cache,activeIDs, ...
            cfg.referenceMaxTime);
        if strcmp(oracle.status,'FEASIBLE')
            referenceCost = oracle.objective;
            oracleStatus = string(oracle.status);
        else
            referenceCost = NaN;
            oracleStatus = string(oracle.status);
        end

        for strategyIndex = 1:numel(strategies)
            for budgetIndex = 1:numel(budgets)
                strategy = strategies{strategyIndex};
                budget = budgets(budgetIndex);
                algorithmSeed = 7200+100*instanceID+10*e+budget;
                [solution,metrics] = RunStrategyFromState( ...
                    strategy,initialSolution,model,cache,activeIDs, ...
                    event,budget,cfg,algorithmSeed);
                if oracleStatus=="FEASIBLE"
                    gap = (solution.Cost-referenceCost)/ ...
                        max(abs(referenceCost),eps);
                else
                    gap = NaN;
                end
                rows{end+1,1} = {string(level),instanceID, ...
                    scenarioSeed,algorithmSeed,e,event.time, ...
                    string(event.type),numel(event.customerIDs), ...
                    string(strategy),budget,oracleStatus,referenceCost, ...
                    cacheStats.reusedCount,cacheStats.computedCount, ...
                    metrics.warmStartFE,metrics.routingFE,metrics.totalFE, ...
                    metrics.responseTime,solution.Cost,solution.Detail.distance, ...
                    solution.Detail.totalLate,solution.Detail.totalObstacleViolation, ...
                    solution.Detail.isFeasible,gap, ...
                    ComputeRouteDisruption(initialSolution.Detail.routeIDs, ...
                    solution.Detail.routeIDs)}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s instance %d completed.\n',level,instanceID);
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','algorithm_seed','event_index', ...
    'event_time','event_type','event_size','strategy','budget', ...
    'oracle_status','reference_cost','reused_legs','computed_legs', ...
    'warm_start_fe','routing_fe','total_fe','response_time','fitness', ...
    'distance','total_late','obstacle_violation','is_feasible', ...
    'gap_to_reference','route_disruption'});
writetable(summary,fullfile(cfg.outputDir, ...
    'accepted_paired_summary.csv'));
save(fullfile(cfg.outputDir,'accepted_paired_result.mat'), ...
    'summary','manifest','strategies','budgets','cfg');

fprintf('\nSaved leakage-free accepted paired diagnostic results.\n');
end

function model = ApplyEvent(model,event)
if strcmpi(event.type,'add')
    for k = 1:numel(event.customers)
        customer = event.customers(k);
        id = customer.id;
        groundZ = interp2(model.X,model.Y,model.terrainZ, ...
            customer.xy(1),customer.xy(2),'linear');
        model.customerXY(id,:) = customer.xy;
        model.customerXYZ(id,:) = [customer.xy,model.flightAltitude];
        model.customerGroundXYZ(id,:) = [customer.xy,groundZ];
        model.windows(id,:) = customer.window;
        model.service(id,1) = customer.service;
    end
end
model.customerIDs = (1:size(model.customerXY,1))';
model.nCustomers = numel(model.customerIDs);
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
metrics.responseTime = metrics.warmStartTime+metrics.routingTime;
end
