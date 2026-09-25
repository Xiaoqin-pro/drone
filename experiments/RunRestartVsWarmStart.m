clc
clear
close all

%% Restart-PSO 与 Warm-start-PSO 动态对比
root = fileparts(mfilename('fullpath'));
cfg.seed = 2026;
cfg.maxIterations = 120;
cfg.population = 30;
cfg.inertia = 0.90;
cfg.inertiaDamp = 0.995;
cfg.c1 = 1.7;
cfg.c2 = 1.7;
cfg.outputDir = fullfile(root,'results');

if ~exist(cfg.outputDir,'dir')
    mkdir(cfg.outputDir);
end

%% 生成相同初始场景和事件
rng(cfg.seed,'twister');
baseModel = CreateModel();
baseModel.cfg = cfg;
baseModel.cfg.silent = true;
events = CreateDynamicEvents(baseModel);
activeIDs = baseModel.customerIDs(:)';
initialCache = BuildLegCache(baseModel,activeIDs,baseModel.homePosition);

% 两种动态策略共享同一条初始路线，保证事件前状态一致。
rng(cfg.seed,'twister');
[initialSolution,initialBestCost,initialStats] = RoutingPSO( ...
    baseModel,initialCache,cfg.maxIterations,cfg.population, ...
    cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

fprintf('Initial route: ');
fprintf('C%d ',initialSolution.Detail.routeIDs);
fprintf('\n');

restartResult = RunStrategy('Restart',baseModel,events, ...
    initialSolution,initialCache,cfg);
warmResult = RunStrategy('WarmStart',baseModel,events, ...
    initialSolution,initialCache,cfg);

summary = [restartResult;warmResult];
writetable(summary,fullfile(cfg.outputDir, ...
    'restart_vs_warmstart_summary.csv'));
save(fullfile(cfg.outputDir,'restart_vs_warmstart_result.mat'), ...
    'summary','events','initialSolution','initialBestCost', ...
    'initialStats','restartResult','warmResult','cfg');

%% 绘制动态响应时间和重规划时间比较
f = figure('Visible','off','Color','w');
response = [summary.response_time];
routeTime = [summary.routing_time];
bar([response;routeTime]');
set(gca,'XTick',1:numel(response),'XTickLabel', ...
    summary.strategy+"-E"+string(summary.event_index));
ylabel('Time (seconds)');
title('Restart PSO versus warm-start PSO');
legend('Total response time','Routing PSO time','Location','northwest');
grid on;
exportgraphics(f,fullfile(cfg.outputDir, ...
    'restart_vs_warmstart_time.png'),'Resolution',180);

fprintf('\n===== Restart vs Warm-start =====\n');
for k = 1:height(summary)
    fprintf('%s Event %d: response %.3fs, routing %.3fs, ', ...
        summary.strategy(k),summary.event_index(k), ...
        summary.response_time(k),summary.routing_time(k));
    fprintf('initial %.2f, final %.2f, first feasible iteration %d, FE %d\n', ...
        summary.initial_best_fitness(k),summary.fitness(k), ...
        summary.first_feasible_iteration(k),summary.function_evaluations(k));

end
function summary = RunStrategy(strategy,baseModel,events, ...
    initialSolution,initialCache,cfg)
model = baseModel;
cache = initialCache;
solution = initialSolution;
activeIDs = model.customerIDs(:)';
rows = cell(numel(events),1);

for e = 1:numel(events)
    event = events(e);
    state = ExecuteUntilEvent(solution,model,event.time);
    completedIDs = state.completedIDs(:)';
    activeIDs = setdiff(activeIDs,completedIDs,'stable');

    if strcmpi(event.type,'cancel')
        activeIDs = setdiff(activeIDs,event.customerID,'stable');
        model = ApplyDynamicEvent(model,event);
        eventCustomer = event.customerID;
    else
        model = ApplyDynamicEvent(model,event);
        activeIDs = [activeIDs,event.customer.id];
        eventCustomer = event.customer.id;
    end

    model.startTime = event.time;
    model.depot = state.position;
    [cache,cacheStats] = UpdateLegCache(model,cache, ...
        activeIDs,state.position);

    if strcmpi(strategy,'WarmStart')
        rng(cfg.seed+5000*e,'twister');
        warmTimer = tic;
        [initialPositions,warmInfo] = BuildWarmStartPopulation( ...
            solution,activeIDs,model,cache,cfg.population);
        warmStartTime = toc(warmTimer);
    else
        initialPositions = [];
        warmInfo.candidateCount = 0;
        warmStartTime = 0;
    end

    if strcmpi(strategy,'WarmStart')
        rng(cfg.seed+7000*e,'twister');
    else
        rng(cfg.seed+8000*e,'twister');
    end
    routeTimer = tic;
    [solution,~,routingStats] = RoutingPSO(model,cache, ...
        cfg.maxIterations,cfg.population,cfg.inertia, ...
        cfg.inertiaDamp,cfg.c1,cfg.c2,initialPositions);
    routingTime = toc(routeTimer);

    responseTime = cacheStats.onlineLegTime+warmStartTime+routingTime;
    rows{e} = {string(strategy),e,event.time,string(event.type), ...
        eventCustomer,cacheStats.reusedCount,cacheStats.computedCount, ...
        cacheStats.onlineLegTime,warmStartTime,routingTime,responseTime, ...
        routingStats.initialBestCost,solution.Cost,solution.Detail.distance,solution.Detail.totalLate, ...
        solution.Detail.totalObstacleViolation, ...
        routingStats.firstFeasibleIteration, ...
        routingStats.functionEvaluations,warmInfo.candidateCount};
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'strategy','event_index','event_time','event_type','event_customer', ...
    'reused_legs','computed_legs','online_leg_time','warm_start_time', ...
    'routing_time','response_time','initial_best_fitness','fitness','distance','total_late', ...
    'obstacle_violation','first_feasible_iteration', ...
    'function_evaluations','warm_candidate_count'});
end



