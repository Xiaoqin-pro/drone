function RunPairedDiagnostic
%RUNPAIREDDIAGNOSTIC Benchmark Protocol v1 的配对诊断实验
%   当前默认2个实例用于验收；确认后将nInstances改为10。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
strategies = {'Restart','WarmStart','Repair'};
budgets = [500 1000 1500 2500];
nInstances = 2;

cfg.scenarioSeedBase = 3000;
cfg.algorithmSeedBase = 7000;
cfg.maxInitialIterations = 50;
cfg.population = 30;
cfg.inertia = 0.90;
cfg.inertiaDamp = 0.995;
cfg.c1 = 1.7;
cfg.c2 = 1.7;
cfg.outputDir = fullfile(root,'results');
if ~exist(cfg.outputDir,'dir')
    mkdir(cfg.outputDir);
end

rows = cell(0,1);
feasibilityRows = cell(0,1);

for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = cfg.scenarioSeedBase+instanceIndex;
        rng(scenarioSeed,'twister');
        baseModel = CreateModel();
        baseModel.cfg = cfg;
        baseModel.cfg.silent = true;
        instance = GenerateDynamicInstance(baseModel,scenarioSeed, ...
            levels{levelIndex});
        activeIDs = baseModel.customerIDs(:)';
        cache = BuildLegCache(baseModel,activeIDs,baseModel.homePosition);

        rng(cfg.algorithmSeedBase+instanceIndex,'twister');
        [initialSolution,~,initialStats] = RoutingPSO( ...
            baseModel,cache,cfg.maxInitialIterations,cfg.population, ...
            cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

        % 对初始实例做参考可行性审计。
        reference = VerifyScenarioFeasibility(baseModel,cache, ...
            activeIDs,1000);
        feasibilityRows{end+1,1} = {string(levels{levelIndex}), ...
            instanceIndex,scenarioSeed,reference.isFound, ...
            reference.firstFeasibleFE,reference.referenceCost, ...
            initialStats.functionEvaluations}; %#ok<AGROW>

        for strategyIndex = 1:numel(strategies)
            for budgetIndex = 1:numel(budgets)
                algorithmSeed = cfg.algorithmSeedBase ...
                    + 100*instanceIndex+budgetIndex;
                conditionRows = RunCondition(strategies{strategyIndex}, ...
                    baseModel,instance.events,initialSolution,cache, ...
                    activeIDs,budgets(budgetIndex),cfg,algorithmSeed, ...
                    levelIndex,instanceIndex,scenarioSeed);
                rows = [rows;conditionRows]; %#ok<AGROW>
            end
        end
        fprintf('%s instance %d/%d finished; scenarioSeed=%d.\n', ...
            levels{levelIndex},instanceIndex,nInstances,scenarioSeed);
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','algorithm_seed','strategy', ...
    'budget','event_index','event_time','event_type','event_size', ...
    'reused_legs','computed_legs','online_leg_time','warm_start_time', ...
    'routing_or_repair_time','response_time','warm_start_fe', ...
    'routing_or_repair_fe','total_fe','final_fitness','distance', ...
    'total_late','obstacle_violation','route_disruption', ...
    'is_feasible','first_feasible_fe','reference_found', ...
    'reference_fe','reference_cost','state_x','state_y','state_z','state_time'});
writetable(summary,fullfile(cfg.outputDir, ...
    'paired_diagnostic_summary.csv'));

feasibility = cell2table(vertcat(feasibilityRows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','reference_found', ...
    'reference_first_feasible_fe','reference_cost','initial_fe'});
writetable(feasibility,fullfile(cfg.outputDir, ...
    'paired_feasibility_audit.csv'));
save(fullfile(cfg.outputDir,'paired_diagnostic_result.mat'), ...
    'summary','feasibility','levels','strategies','budgets','cfg');

fprintf('\nSaved paired diagnostic results to: %s\n',cfg.outputDir);
end

function rows = RunCondition(strategy,baseModel,events, ...
    initialSolution,initialCache,initialActiveIDs,budget,cfg, ...
    algorithmSeed,levelIndex,instanceIndex,scenarioSeed)
model = baseModel;
cache = initialCache;
activeIDs = initialActiveIDs;
solution = initialSolution;
rows = cell(numel(events),1);

for e = 1:numel(events)
    event = events(e);
    oldRoute = solution.Detail.routeIDs;
    state = ExecuteUntilEvent(solution,model,event.time);
    completedIDs = state.completedIDs(:)';
    activeIDs = setdiff(activeIDs,completedIDs,'stable');

    if strcmpi(event.type,'cancel')
        activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
    else
        for k = 1:numel(event.customers)
            model = AddEventCustomer(model,event.customers(k));
            activeIDs = [activeIDs,event.customers(k).id]; %#ok<AGROW>
        end
    end

    model.startTime = event.time;
    model.depot = state.position;
    [cache,cacheStats] = UpdateLegCache(model,cache, ...
        activeIDs,state.position);
    reference = VerifyScenarioFeasibility(model,cache,activeIDs,300);

    warmFE = 0;
    warmTime = 0;
    if strcmpi(strategy,'WarmStart')
        rng(algorithmSeed+e,'twister');
        warmTimer = tic;
        [initialPositions,warmInfo] = BuildWarmStartPopulation( ...
            solution,activeIDs,model,cache,cfg.population);
        warmTime = toc(warmTimer);
        warmFE = warmInfo.evaluations;
    else
        initialPositions = [];
    end

    if strcmpi(strategy,'Repair')
        repairTimer = tic;
        [solution,repairInfo] = LocalRepair(solution,activeIDs, ...
            model,cache,event.type,event.customerIDs);
        routeTime = toc(repairTimer);
        routingFE = repairInfo.functionEvaluations;
        if repairInfo.isFeasible
            firstFeasibleFE = routingFE;
        else
            firstFeasibleFE = inf;
        end
    else
        availableFE = max(0,budget-warmFE);
        maxIt = max(0,floor(availableFE/cfg.population)-1);
        rng(algorithmSeed+e*1000+budget,'twister');
        routeTimer = tic;
        [solution,~,stats] = RoutingPSO(model,cache,maxIt, ...
            cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2, ...
            initialPositions);
        routeTime = toc(routeTimer);
        routingFE = stats.functionEvaluations;
        firstFeasibleFE = warmFE+stats.firstFeasibleEvaluation;
    end

    rows{e} = {string(levelName(levelIndex)),instanceIndex, ...
        scenarioSeed,algorithmSeed,string(strategy),budget,e, ...
        event.time,string(event.type),numel(event.customerIDs), ...
        cacheStats.reusedCount,cacheStats.computedCount, ...
        cacheStats.onlineLegTime,warmTime,routeTime, ...
        cacheStats.onlineLegTime+warmTime+routeTime,warmFE, ...
        routingFE,warmFE+routingFE,solution.Cost, ...
        solution.Detail.distance,solution.Detail.totalLate, ...
        solution.Detail.totalObstacleViolation, ...
        ComputeRouteDisruption(oldRoute,solution.Detail.routeIDs), ...
        solution.Detail.isFeasible,firstFeasibleFE, ...
        reference.isFound,reference.firstFeasibleFE,reference.referenceCost, ...
        state.position(1),state.position(2),state.position(3),event.time};
end
end

function model = AddEventCustomer(model,customer)
groundZ = interp2(model.X,model.Y,model.terrainZ, ...
    customer.xy(1),customer.xy(2),'linear');
id = customer.id;
model.customerXY(id,:) = customer.xy;
model.customerXYZ(id,:) = [customer.xy,model.flightAltitude];
model.customerGroundXYZ(id,:) = [customer.xy,groundZ];
model.windows(id,:) = customer.window;
model.service(id,1) = customer.service;
model.customerIDs = (1:size(model.customerXY,1))';
model.nCustomers = numel(model.customerIDs);
end

function name = levelName(index)
names = {'Mild','Moderate','Severe'};
name = names{index};
end



