function RunThreeStrategyBenchmark
%RUNTHREESTRATEGYBENCHMARK Restart / Warm-start / Local Repair公平比较

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
strategies = {'Restart','WarmStart','Repair'};
budgets = [500 1000 1500 2500];
nRuns = 1; % 当前用于诊断；正式实验建议提高到10或30

cfg.seedBase = 6200;
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
for levelIndex = 1:numel(levels)
    for runIndex = 1:nRuns
        % 三种事件强度使用同一环境，只改变订单变化集合。
        rng(cfg.seedBase+runIndex,'twister');
        baseModel = CreateModel();
        baseModel.cfg = cfg;
        baseModel.cfg.silent = true;
        events = CreateBenchmarkEvents(levels{levelIndex});
        activeIDs = baseModel.customerIDs(:)';
        cache = BuildLegCache(baseModel,activeIDs,baseModel.homePosition);
        [initialSolution,~,~] = RoutingPSO(baseModel,cache, ...
            cfg.maxInitialIterations,cfg.population,cfg.inertia, ...
            cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

        for strategyIndex = 1:numel(strategies)
            for budgetIndex = 1:numel(budgets)
                resultRows = RunCondition(strategies{strategyIndex}, ...
                    baseModel,events,initialSolution,cache,activeIDs, ...
                    budgets(budgetIndex),cfg,levelIndex,runIndex);
                rows = [rows;resultRows]; %#ok<AGROW>
            end
        end
        fprintf('%s run %d/%d completed.
', ...
            levels{levelIndex},runIndex,nRuns);
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','run','strategy','budget','event_index','event_time', ...
    'event_type','event_size','reused_legs','computed_legs', ...
    'online_leg_time','warm_start_time','routing_or_repair_time', ...
    'response_time','warm_start_fe','routing_or_repair_fe','total_fe', ...
    'initial_best_fitness','final_fitness','distance','total_late', ...
    'obstacle_violation','route_disruption','is_feasible', ...
    'first_feasible_fe'});
writetable(summary,fullfile(cfg.outputDir, ...
    'three_strategy_benchmark_summary.csv'));
save(fullfile(cfg.outputDir,'three_strategy_benchmark_result.mat'), ...
    'summary','levels','strategies','budgets','cfg');

fprintf('
Saved three-strategy benchmark to:
%s
',cfg.outputDir);
end

function rows = RunCondition(strategy,baseModel,events, ...
    initialSolution,initialCache,initialActiveIDs,budget,cfg, ...
    levelIndex,runIndex)
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
        for id = event.customerIDs
            model = AddBenchmarkCustomer(model,id);
            activeIDs = [activeIDs,id]; %#ok<AGROW>
        end
    end

    model.startTime = event.time;
    model.depot = state.position;
    [cache,cacheStats] = UpdateLegCache(model,cache, ...
        activeIDs,state.position);

    warmFE = 0;
    warmTime = 0;
    if strcmpi(strategy,'WarmStart')
        rng(cfg.seedBase+levelIndex*100+e,'twister');
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
        rng(cfg.seedBase+levelIndex*1000+e*10+budget+runIndex,'twister');
        routeTimer = tic;
        [solution,~,stats] = RoutingPSO(model,cache,maxIt, ...
            cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2, ...
            initialPositions);
        routeTime = toc(routeTimer);
        routingFE = stats.functionEvaluations;
        firstFeasibleFE = warmFE+stats.firstFeasibleEvaluation;
    end

    finalRoute = solution.Detail.routeIDs;
    rows{e} = {string(levelName(levelIndex)),runIndex,string(strategy), ...
        budget,e,event.time,string(event.type),numel(event.customerIDs), ...
        cacheStats.reusedCount,cacheStats.computedCount, ...
        cacheStats.onlineLegTime,warmTime,routeTime, ...
        cacheStats.onlineLegTime+warmTime+routeTime,warmFE, ...
        routingFE,warmFE+routingFE, ...
        NaN,solution.Cost, ...
        solution.Detail.distance,solution.Detail.totalLate, ...
        solution.Detail.totalObstacleViolation, ...
        ComputeRouteDisruption(oldRoute,finalRoute), ...
        solution.Detail.isFeasible,firstFeasibleFE};
end
end

function cost = previousCost(solution,oldRoute,model,cache)
% 仅用于输出旧方案规模，不作为算法比较指标。
if isempty(oldRoute)
    cost = NaN;
else
    [cost,~] = EvaluateSchedule(oldRoute,model,cache);
end
end

function events = CreateBenchmarkEvents(level)
switch lower(level)
    case 'mild'
        cancelIDs = [18]; addIDs = [21];
    case 'moderate'
        cancelIDs = [15 18]; addIDs = [21 22];
    case 'severe'
        cancelIDs = [11 12 15 18]; addIDs = [21 22 23 24];
    otherwise
        error('Unknown level: %s',level);
end
events(1).time = 90;
events(1).type = 'cancel';
events(1).customerIDs = cancelIDs;
events(2).time = 150;
events(2).type = 'add';
events(2).customerIDs = addIDs;
end

function name = levelName(index)
names = {'Mild','Moderate','Severe'};
name = names{index};
end

function model = AddBenchmarkCustomer(model,id)
newXY = [80+70*mod(id-21,4),170+180*floor((id-21)/4)];
newWindow = [100+5*id,280+5*id];
groundZ = interp2(model.X,model.Y,model.terrainZ,newXY(1),newXY(2),'linear');
model.customerXY(id,:) = newXY;
model.customerXYZ(id,:) = [newXY,model.flightAltitude];
model.customerGroundXYZ(id,:) = [newXY,groundZ];
model.windows(id,:) = newWindow;
model.service(id,1) = 10;
model.customerIDs = (1:size(model.customerXY,1))';
model.nCustomers = numel(model.customerIDs);
end



