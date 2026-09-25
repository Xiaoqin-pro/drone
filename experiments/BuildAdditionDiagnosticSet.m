function BuildAdditionDiagnosticSet
%BUILDADDITIONDIAGNOSTICSET 构造30个单事件新增订单诊断实例
%   10个M=1、10个M=2、10个M=4；实例保存已知witness route。

clc
clear
close all

scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
levels = {'M1','M2','M4'};
counts = [1 2 4];
nInstances = 10;
seedBase = 6000;
outputDir = fullfile(root,'results','addition_instances');
if ~exist(outputDir,'dir')
    mkdir(outputDir);
end

% 使用宽松但固定的基础时间窗，保证新增事件研究重点是任务到达和插入影响。
baseModel = CreateModel();
baseModel.windows(:,2) = baseModel.windows(:,2)+300;
baseModel.cfg.silent = true;
activeInitial = baseModel.customerIDs(:)';
baseCache = BuildLegCache(baseModel,activeInitial,baseModel.homePosition);
rng(860000,'twister');
[baseSolution,~,~] = RoutingPSO(baseModel,baseCache,100,40, ...
    0.90,0.995,1.7,1.7,[]);
if ~baseSolution.Detail.isFeasible
    error('Base reference route is not feasible; widen base windows before construction.');
end

rows = cell(0,1);
for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = seedBase+100*levelIndex+instanceIndex;
        rng(scenarioSeed,'twister');
        M = counts(levelIndex);
        model = baseModel;
        eventTime = 90;
        state = ExecuteUntilEvent(baseSolution,model,eventTime);
        activeIDs = setdiff(activeInitial,state.completedIDs,'stable');
        route = baseSolution.Detail.routeIDs( ...
            ismember(baseSolution.Detail.routeIDs,activeIDs));
        model.startTime = eventTime;
        model.depot = state.position;

        addedCustomers = repmat(struct('id',0,'xy',[0 0], ...
            'window',[0 0],'service',10),M,1);
        nextID = model.nCustomers+1;
        for k = 1:M
            insertPosition = randi(numel(route)+1);
            if insertPosition==1
                from = state.position;
            else
                from = model.customerXYZ(route(insertPosition-1),:);
            end
            if insertPosition>numel(route)
                to = model.homePosition;
            else
                to = model.customerXYZ(route(insertPosition),:);
            end
            xy = MidpointXY(from,to,model);
            id = nextID+k-1;
            addedCustomers(k).id = id;
            addedCustomers(k).xy = xy;
            addedCustomers(k).window = [0 2000];
            addedCustomers(k).service = 8+4*rand;
            model = AddCustomer(model,addedCustomers(k));
            activeIDs = [activeIDs,id]; %#ok<AGROW>
            route = [route(1:insertPosition-1),id, ...
                route(insertPosition:end)];
        end

        cache = BuildLegCache(model,activeIDs,state.position);
        [referenceCost,referenceDetail] = EvaluateSchedule(route,model,cache);
        if ~referenceDetail.isFeasible
            % 保留宽松窗口并用路由级PSO修正访问顺序，仍保存可行witness。
            [referenceSolution,~,~] = RoutingPSO(model,cache,120,40, ...
                0.90,0.995,1.7,1.7,[]);
            if ~referenceSolution.Detail.isFeasible
                error('Could not construct feasible addition instance seed=%d.',scenarioSeed);
            end
            route = referenceSolution.Detail.routeIDs;
            referenceCost = referenceSolution.Cost;
            referenceDetail = referenceSolution.Detail;
        end

        slack = SlackForLevel(levels{levelIndex});
        for k = 1:M
            id = addedCustomers(k).id;
            hit = find(referenceDetail.records(:,1)==id,1);
            serviceStart = referenceDetail.records(hit,3);
            addedCustomers(k).window = [max(0,serviceStart-slack), ...
                serviceStart+slack];
            model.windows(id,:) = addedCustomers(k).window;
        end
        [referenceCost,referenceDetail] = EvaluateSchedule(route,model,cache);
        if ~referenceDetail.isFeasible
            error('Tightened witness became infeasible, seed=%d.',scenarioSeed);
        end

        event.type = 'add';
        event.time = eventTime;
        event.customerIDs = [addedCustomers.id];
        event.customers = addedCustomers;
        instance.level = levels{levelIndex};
        instance.instance = instanceIndex;
        instance.scenarioSeed = scenarioSeed;
        instance.event = event;
        instance.state = state;
        instance.activeIDs = activeIDs;
        instance.referenceRoute = route;
        instance.referenceSchedule = referenceDetail.records;
        instance.referenceCost = referenceCost;
        instance.model = model;
        instance.cache = cache;
        fileName = sprintf('addition_%s_%02d.mat',levels{levelIndex},instanceIndex);
        save(fullfile(outputDir,fileName),'instance');
        rows{end+1,1} = {string(levels{levelIndex}),instanceIndex, ...
            scenarioSeed,fileName,referenceCost,referenceDetail.totalLate}; %#ok<AGROW>
    end
    fprintf('%s: generated %d instances.\n',levels{levelIndex},nInstances);
end

manifest = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','file_name','reference_cost', ...
    'reference_late'});
writetable(manifest,fullfile(outputDir,'addition_manifest.csv'));
disp(manifest);
end

function xy = MidpointXY(from,to,model)
mid = from(1:2)+0.5*(to(1:2)-from(1:2));
delta = to(1:2)-from(1:2);
if norm(delta)<eps
    perpendicular = [0 0];
else
    perpendicular = [-delta(2),delta(1)]/norm(delta);
end
xy = mid+0.08*norm(delta)*(2*rand-1)*perpendicular;
xy(1) = min(max(xy(1),min(model.x)+20),max(model.x)-20);
xy(2) = min(max(xy(2),min(model.y)+20),max(model.y)-20);
end

function model = AddCustomer(model,customer)
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

function slack = SlackForLevel(level)
switch level
    case 'M1'
        slack = 120;
    case 'M2'
        slack = 80;
    case 'M4'
        slack = 40;
end
end

