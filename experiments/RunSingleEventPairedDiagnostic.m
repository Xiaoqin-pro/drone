function RunSingleEventPairedDiagnostic
%RUNSINGLEEVENTPAIREDDIAGNOSTIC 单事件共同状态配对诊断
%   每个事件单独实验，Restart/Warm/Repair从完全相同的事件状态出发。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
strategies = {'Restart','WarmStart','Repair'};
budgets = [500 1000 1500 2500];
nInstances = 2; % 当前验收；通过后改为10
cfg.scenarioSeedBase = 3000;
cfg.algorithmSeedBase = 7200;
cfg.referenceMaxTime = 10;
cfg.population = 30;
cfg.inertia = 0.90;
cfg.inertiaDamp = 0.995;
cfg.c1 = 1.7;
cfg.c2 = 1.7;
cfg.initialIterations = 50;
cfg.outputDir = fullfile(root,'results');

rows = cell(0,1);
for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = cfg.scenarioSeedBase+instanceIndex;
        rng(scenarioSeed,'twister');
        baseModel = CreateModel();
        baseModel.cfg.silent = true;
        instance = GenerateDynamicInstance(baseModel,scenarioSeed, ...
            levels{levelIndex});
        activeInitial = baseModel.customerIDs(:)';
        initialCache = BuildLegCache(baseModel,activeInitial, ...
            baseModel.homePosition);
        rng(cfg.algorithmSeedBase+instanceIndex,'twister');
        [initialSolution,~,~] = RoutingPSO(baseModel,initialCache, ...
            cfg.initialIterations,cfg.population,cfg.inertia, ...
            cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

        for eventIndex = 1:numel(instance.events)
            % 每个事件都从同一个初始计划重新模拟，形成共同事件状态。
            event = instance.events(eventIndex);
            state = ExecuteUntilEvent(initialSolution,baseModel,event.time);
            activeIDs = setdiff(activeInitial,state.completedIDs,'stable');
            model = baseModel;
            model = ApplyEvent(model,event);
            if strcmpi(event.type,'cancel')
                activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
            else
                activeIDs = [activeIDs,event.customerIDs]; %#ok<AGROW>
            end
            model.startTime = event.time;
            model.depot = state.position;
            [cache,cacheStats] = UpdateLegCache(model,initialCache, ...
                activeIDs,state.position);
            oracle = ExactTWFeasibilityOracle(model,cache,activeIDs, ...
                cfg.referenceMaxTime);

            for strategyIndex = 1:numel(strategies)
                for budgetIndex = 1:numel(budgets)
                    strategy = strategies{strategyIndex};
                    budget = budgets(budgetIndex);
                    [solution,metrics] = RunStrategyFromState( ...
                        strategy,initialSolution,model,cache,activeIDs, ...
                        event,budget,cfg,scenarioSeed,instanceIndex, ...
                        eventIndex);
                    if strcmp(oracle.status,'FEASIBLE')
                        gap = (solution.Cost-oracle.objective)/ ...
                            max(abs(oracle.objective),eps);
                    else
                        gap = NaN;
                    end
                    rows{end+1,1} = {string(levels{levelIndex}), ...
                        instanceIndex,scenarioSeed,metrics.algorithmSeed, ...
                        eventIndex,event.time,string(event.type), ...
                        numel(event.customerIDs),string(strategy),budget, ...
                        string(oracle.status),oracle.objective, ...
                        cacheStats.reusedCount,cacheStats.computedCount, ...
                        metrics.warmStartFE,metrics.routingFE,metrics.totalFE, ...
                        metrics.responseTime,solution.Cost, ...
                        solution.Detail.distance,solution.Detail.totalLate, ...
                        solution.Detail.totalObstacleViolation, ...
                        solution.Detail.isFeasible,gap, ...
                        ComputeRouteDisruption(initialSolution.Detail.routeIDs, ...
                        solution.Detail.routeIDs)}; %#ok<AGROW>
                end
            end
        end
        fprintf('%s instance %d/%d finished.\n', ...
            levels{levelIndex},instanceIndex,nInstances);
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','algorithm_seed','event_index', ...
    'event_time','event_type','event_size','strategy','budget', ...
    'oracle_status','reference_cost','reused_legs','computed_legs', ...
    'warm_start_fe','routing_fe','total_fe','response_time','fitness', ...
    'distance','total_late','obstacle_violation','is_feasible', ...
    'gap_to_reference','route_disruption'});
writetable(summary,fullfile(cfg.outputDir, ...
    'single_event_paired_summary.csv'));
save(fullfile(cfg.outputDir,'single_event_paired_result.mat'), ...
    'summary','levels','strategies','budgets','cfg');

fprintf('\nSaved single-event paired diagnostic results to: %s\n',cfg.outputDir);
end

function [solution,metrics] = RunStrategyFromState(strategy,previousSolution, ...
    model,cache,activeIDs,event,budget,cfg,scenarioSeed,instanceIndex,eventIndex)
metrics.algorithmSeed = cfg.algorithmSeedBase+100*instanceIndex ...
    +10*eventIndex+budget;
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
        rng(metrics.algorithmSeed,'twister');
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
    rng(metrics.algorithmSeed,'twister');
    timer = tic;
    [solution,~,stats] = RoutingPSO(model,cache,maxIt,cfg.population, ...
        cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,initialPositions);
    metrics.routingTime = toc(timer);
    metrics.routingFE = stats.functionEvaluations;
end
metrics.totalFE = metrics.warmStartFE+metrics.routingFE;
metrics.responseTime = metrics.routingTime+metrics.warmStartTime;
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
