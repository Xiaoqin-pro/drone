clc
clear
close all

%% 动态分层基线参数
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

%% 初始环境、事件和初始航段缓存
rng(cfg.seed,'twister');
model = CreateModel();
model.cfg = cfg;
model.cfg.silent = true;
events = CreateDynamicEvents(model);
activeIDs = model.customerIDs(:)';
cache = BuildLegCache(model,activeIDs,model.homePosition);

fprintf('Initial leg cache: %d computed legs.\n',cache.newCount);

%% 初始路由规划
initialModel = model;
initialModel.startTime = 0;
initialTimer = tic;
[solution,initialBestCost] = RoutingPSO(initialModel,cache, ...
    cfg.maxIterations,cfg.population,cfg.inertia,cfg.inertiaDamp, ...
    cfg.c1,cfg.c2);
initialRoutingTime = toc(initialTimer);

solutions = cell(numel(events)+1,1);
solutions{1} = solution;
states = cell(numel(events),1);
eventRows = cell(numel(events),1);

fprintf('Initial route: ');
fprintf('C%d ',solution.Detail.routeIDs);
fprintf('\n');

%% 事件驱动的动态重规划
for e = 1:numel(events)
    event = events(e);
    fprintf('\n===== Event %d: %s at t = %.2f =====\n', ...
        e,event.type,event.time);

    % 允许事件发生在某一条三维航段的中间位置。
    state = ExecuteUntilEvent(solution,model,event.time);
    states{e} = state;

    completedIDs = state.completedIDs(:)';
    activeIDs = setdiff(activeIDs,completedIDs,'stable');
    oldActiveIDs = activeIDs;

    if strcmpi(event.type,'cancel')
        if ismember(event.customerID,activeIDs)
            activeIDs = setdiff(activeIDs,event.customerID,'stable');
        end
        model = ApplyDynamicEvent(model,event);
        fprintf('Cancelled order: C%d\n',event.customerID);
    elseif strcmpi(event.type,'add')
        model = ApplyDynamicEvent(model,event);
        activeIDs = [activeIDs,event.customer.id];
        fprintf('Added order: C%d\n',event.customer.id);
    end

    fprintf('Completed before event: ');
    fprintf('C%d ',completedIDs);
    fprintf('\nRemaining active orders: ');
    fprintf('C%d ',activeIDs);
    fprintf('\n');

    model.startTime = event.time;
    model.depot = state.position;

    % 已有客户-客户/客户-home航段尽量复用；
    % 当前执行位置和新增客户相关航段按需计算。
    [cache,cacheStats] = UpdateLegCache(model,cache,activeIDs,state.position);

    rng(cfg.seed+100*e,'twister');
    timer = tic;
    [solution,bestCost] = RoutingPSO(model,cache,cfg.maxIterations, ...
        cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2);
    routingTime = toc(timer);
    solutions{e+1} = solution;

    responseTime = cacheStats.onlineLegTime+routingTime;
    eventRows{e} = {e,event.time,string(event.type), ...
        eventCustomerID(event),joinNumbers(completedIDs), ...
        joinNumbers(oldActiveIDs),joinNumbers(activeIDs), ...
        cacheStats.reusedCount,cacheStats.computedCount, ...
        cacheStats.onlineLegTime,routingTime,responseTime, ...
        solution.Cost,solution.Detail.distance,solution.Detail.totalLate, ...
        solution.Detail.totalObstacleViolation};

    fprintf('Replanned route: ');
    fprintf('C%d ',solution.Detail.routeIDs);
    fprintf('\nReuse %d legs, recompute %d legs, ', ...
        cacheStats.reusedCount,cacheStats.computedCount);
    fprintf('online leg time %.3fs, routing %.3fs, response %.3fs.\n', ...
        cacheStats.onlineLegTime,routingTime,responseTime);
end

%% 保存动态结果
rows = vertcat(eventRows{:});
summary = cell2table(rows,'VariableNames',{ ...
    'event_index','event_time','event_type','event_customer', ...
    'completed_ids','old_active_ids','new_active_ids', ...
    'reused_legs','computed_legs','online_leg_time', ...
    'routing_time','response_time','fitness','distance', ...
    'total_late','obstacle_violation'});
writetable(summary,fullfile(cfg.outputDir, ...
    'hierarchical_dynamic_summary.csv'));
save(fullfile(cfg.outputDir,'hierarchical_dynamic_result.mat'), ...
    'model','events','solutions','states','summary','cache', ...
    'initialBestCost','initialRoutingTime','cfg');

PlotDynamicHierarchical(model,solutions,states,events,cfg);

fprintf('\nSaved hierarchical dynamic baseline to: %s\n',cfg.outputDir);

function value = eventCustomerID(event)
if strcmpi(event.type,'cancel')
    value = event.customerID;
else
    value = event.customer.id;
end
end

function text = joinNumbers(values)
if isempty(values)
    text = "";
else
    text = join(string(values),',');
end
end
