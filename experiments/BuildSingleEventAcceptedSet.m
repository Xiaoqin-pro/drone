function BuildSingleEventAcceptedSet
%BUILDSINGLEEVENTACCEPTEDSET 生成与单事件共同状态协议一致的可行实例
%   每个实例只包含一个事件：cancel 或 add。
%   参考解只用于实例验收和离线Gap，不作为策略输入。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
eventTypes = {'cancel','add'};
nInstances = 2;
seedBase = 4000;
maxAttempts = 12;
outputDir = fullfile(root,'results');
rows = cell(0,1);

for levelIndex = 1:numel(levels)
    for typeIndex = 1:numel(eventTypes)
        accepted = 0;
        attempt = 0;
        while accepted<nInstances && attempt<maxAttempts*nInstances
            attempt = attempt+1;
            scenarioSeed = seedBase+1000*levelIndex+100*typeIndex+attempt;
            baseModel = CreateModel();
            baseModel.cfg.silent = true;
            candidate = GenerateDynamicInstance(baseModel, ...
                scenarioSeed,levels{levelIndex});
            if strcmp(eventTypes{typeIndex},'cancel')
                event = candidate.events(1);
                eventIndex = 1;
            else
                event = candidate.events(2);
                eventIndex = 2;
                % 新增订单先使用宽窗口，便于构造可行参考状态。
                for k = 1:numel(event.customers)
                    event.customers(k).window = [0 2000];
                end
            end

            % 事件前统一初始规划，只用于确定共同事件状态。
            activeInitial = baseModel.customerIDs(:)';
            cache0 = BuildLegCache(baseModel,activeInitial, ...
                baseModel.homePosition);
            rng(880000+scenarioSeed,'twister');
            [initialSolution,~,~] = RoutingPSO(baseModel,cache0, ...
                50,30,0.90,0.995,1.7,1.7,[]);
            state = ExecuteUntilEvent(initialSolution,baseModel,event.time);
            activeIDs = setdiff(activeInitial,state.completedIDs,'stable');
            model = ApplySingleEvent(baseModel,event);
            if strcmpi(event.type,'cancel')
                activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
            else
                activeIDs = [activeIDs,event.customerIDs]; %#ok<AGROW>
            end
            model.startTime = event.time;
            model.depot = state.position;
            [cache,~] = UpdateLegCache(model,cache0,activeIDs,state.position);

            oracle = ExactTWFeasibilityOracle(model,cache,activeIDs,20);
            if ~strcmp(oracle.status,'FEASIBLE')
                fprintf('Reject %s %s seed=%d: oracle=%s\n', ...
                    levels{levelIndex},eventTypes{typeIndex}, ...
                    scenarioSeed,oracle.status);
                continue;
            end

            referenceRoute = oracle.referenceRoute;
            referenceCost = oracle.objective;

            % 新增订单：依据参考路线的服务时间收紧时间窗，并再次审计。
            if strcmpi(event.type,'add')
                slack = EventSlack(levels{levelIndex});
                detail = EvaluateSchedule(referenceRoute,model,cache);
                for k = 1:numel(event.customers)
                    id = event.customers(k).id;
                    hit = find(detail.records(:,1)==id,1);
                    if isempty(hit)
                        continue;
                    end
                    serviceStart = detail.records(hit,3);
                    newWindow = [max(0,serviceStart-slack), ...
                        serviceStart+slack];
                    event.customers(k).window = newWindow;
                    model.windows(id,:) = newWindow;
                end
                cache = BuildLegCache(model,activeIDs,state.position);
                oracle = ExactTWFeasibilityOracle(model,cache,activeIDs,20);
                if ~strcmp(oracle.status,'FEASIBLE')
                    fprintf('Reject tightened %s %s seed=%d: oracle=%s\n', ...
                        levels{levelIndex},eventTypes{typeIndex}, ...
                        scenarioSeed,oracle.status);
                    continue;
                end
                referenceRoute = oracle.referenceRoute;
                referenceCost = oracle.objective;
            end

            accepted = accepted+1;
            instance = struct();
            instance.level = string(levels{levelIndex});
            instance.eventType = string(event.type);
            instance.eventIndex = eventIndex;
            instance.scenarioSeed = scenarioSeed;
            instance.event = event;
            instance.initialRoute = initialSolution.Detail.routeIDs;
            instance.state = state;
            instance.activeIDs = activeIDs;
            instance.referenceRoute = referenceRoute;
            instance.referenceCost = referenceCost;
            instance.model = model;
            instance.cache = cache;
            instance.oracleStatus = string(oracle.status);

            fileName = sprintf('single_%s_%s_%02d.mat', ...
                levels{levelIndex},eventTypes{typeIndex},accepted);
            save(fullfile(outputDir,fileName),'instance');
            rows{end+1,1} = {string(levels{levelIndex}), ...
                string(event.type),accepted,scenarioSeed,attempt, ...
                fileName,referenceCost}; %#ok<AGROW>
            fprintf('Accepted %s %s %d: seed=%d, attempt=%d.\n', ...
                levels{levelIndex},eventTypes{typeIndex},accepted, ...
                scenarioSeed,attempt);
        end
        if accepted<nInstances
            error('Could not build enough %s %s instances.', ...
                levels{levelIndex},eventTypes{typeIndex});
        end
    end
end

manifest = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','event_type','instance','scenario_seed','attempt', ...
    'file_name','reference_cost'});
writetable(manifest,fullfile(outputDir, ...
    'single_event_benchmark_manifest.csv'));
disp(manifest);
end

function model = ApplySingleEvent(model,event)
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

function slack = EventSlack(level)
switch lower(level)
    case 'mild'
        slack = 120;
    case 'moderate'
        slack = 80;
    case 'severe'
        slack = 40;
    otherwise
        slack = 80;
end
end
