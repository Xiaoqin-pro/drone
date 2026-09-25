function RunDynamicBenchmark
%RUNDYNAMICBENCHMARK 20任务动态基准实验

clc
clear
close all

%% 参数
root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
strategies = {'Restart','WarmStart'};
budgets = [10 20 30 50];
nRuns = 1; % 当前先做结构验证；正式实验建议至少30次

cfg.seedBase = 5200;
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
        rng(cfg.seedBase+runIndex,'twister');
        baseModel = ConfigureBenchmarkLevel(CreateModel(),levels{levelIndex});
        baseModel.cfg = cfg;
        baseModel.cfg.silent = true;
        events = CreateBenchmarkEvents(levels{levelIndex});
        activeIDs = baseModel.customerIDs(:)';
        cache = BuildLegCache(baseModel,activeIDs,baseModel.homePosition);

        [initialSolution,~,initialStats] = RoutingPSO( ...
            baseModel,cache,cfg.maxInitialIterations,cfg.population, ...
            cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

        for strategyIndex = 1:numel(strategies)
            for budgetIndex = 1:numel(budgets)
                [conditionRows,~] = RunOneCondition( ...
                    strategies{strategyIndex},baseModel,events, ...
                    initialSolution,cache,activeIDs,budgets(budgetIndex), ...
                    cfg,levelIndex,runIndex);
                rows = [rows;conditionRows]; %#ok<AGROW>
            end
        end
        fprintf('%s run %d/%d completed. Initial FE = %d.\n', ...
            levels{levelIndex},runIndex,nRuns,initialStats.functionEvaluations);
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','run','strategy','budget','event_index','event_time', ...
    'event_type','event_size','reused_legs','computed_legs', ...
    'online_leg_time','warm_start_time','routing_time','response_time', ...
    'warm_start_fe','routing_fe','total_fe','initial_best_fitness', ...
    'final_fitness','distance','total_late','obstacle_violation', ...
    'first_feasible_iteration','first_feasible_fe'});
writetable(summary,fullfile(cfg.outputDir,'dynamic_benchmark20_summary.csv'));
save(fullfile(cfg.outputDir,'dynamic_benchmark20_result.mat'), ...
    'summary','levels','strategies','budgets','cfg');

[G,levelGroup,strategyGroup,budgetGroup,eventGroup] = findgroups( ...
    summary.level,summary.strategy,summary.budget,summary.event_index);
meanTable = table(levelGroup,strategyGroup,budgetGroup,eventGroup, ...
    splitapply(@mean,summary.response_time,G), ...
    splitapply(@mean,summary.total_fe,G), ...
    splitapply(@mean,summary.final_fitness,G), ...
    splitapply(@mean,summary.distance,G), ...
    splitapply(@mean,summary.total_late,G), ...
    splitapply(@mean,summary.first_feasible_fe,G), ...
    'VariableNames',{'level','strategy','budget','event_index', ...
    'mean_response_time','mean_total_fe','mean_fitness','mean_distance', ...
    'mean_late','mean_first_feasible_fe'});
writetable(meanTable,fullfile(cfg.outputDir,'dynamic_benchmark20_means.csv'));

fprintf('\nSaved 20-task dynamic benchmark results to:\n%s\n',cfg.outputDir);
end

function model = ConfigureBenchmarkLevel(model,level)
switch lower(level)
    case 'mild'
        model.speed = 28;
        model.windows(:,2) = model.windows(:,2)+20;
    case 'moderate'
        model.speed = 25;
        model.windows(:,2) = model.windows(:,2)-60;
    case 'severe'
        model.speed = 21;
        model.windows(:,2) = max(model.windows(:,1)+70, ...
            model.windows(:,2)-160);
        model.obstacles(end+1) = CreateObstacle(model,210,475,115,130,165);
        model.obstacles(end+1) = CreateObstacle(model,650,190,130,110,145);
        model.nObstacles = numel(model.obstacles);
    otherwise
        error('Unknown level: %s',level);
end
end

function obstacle = CreateObstacle(model,cx,cy,width,depth,height)
ground = interp2(model.X,model.Y,model.terrainZ,cx,cy,'linear');
obstacle.xMin = cx-width/2; obstacle.xMax = cx+width/2;
obstacle.yMin = cy-depth/2; obstacle.yMax = cy+depth/2;
obstacle.zMin = ground; obstacle.zMax = ground+height;
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

function [rows,solution] = RunOneCondition(strategy,baseModel,events, ...
    initialSolution,initialCache,initialActiveIDs,budget,cfg, ...
    levelIndex,runIndex)
model = baseModel;
cache = initialCache;
activeIDs = initialActiveIDs;
solution = initialSolution;
rows = cell(numel(events),1);

for e = 1:numel(events)
    event = events(e);
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

    if strcmpi(strategy,'WarmStart')
        rng(cfg.seedBase+levelIndex*100+e,'twister');
        warmTimer = tic;
        [initialPositions,warmInfo] = BuildWarmStartPopulation( ...
            solution,activeIDs,model,cache,cfg.population);
        warmTime = toc(warmTimer);
        warmFE = warmInfo.evaluations;
    else
        initialPositions = [];
        warmTime = 0;
        warmFE = 0;
    end

    rng(cfg.seedBase+levelIndex*1000+e*10+budget+runIndex,'twister');
    routeTimer = tic;
    [solution,~,stats] = RoutingPSO(model,cache,budget,cfg.population, ...
        cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,initialPositions);
    routingTime = toc(routeTimer);

    rows{e} = {string(levelName(levelIndex)),runIndex,string(strategy), ...
        budget,e,event.time,string(event.type),numel(event.customerIDs), ...
        cacheStats.reusedCount,cacheStats.computedCount, ...
        cacheStats.onlineLegTime,warmTime,routingTime, ...
        cacheStats.onlineLegTime+warmTime+routingTime,warmFE, ...
        stats.functionEvaluations,warmFE+stats.functionEvaluations, ...
        stats.initialBestCost,solution.Cost,solution.Detail.distance, ...
        solution.Detail.totalLate,solution.Detail.totalObstacleViolation, ...
        stats.firstFeasibleIteration,stats.firstFeasibleEvaluation};
end
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
