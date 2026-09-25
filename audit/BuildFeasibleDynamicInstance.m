function [instance,audit] = BuildFeasibleDynamicInstance( ...
    baseModel,scenarioSeed,level,referenceBudget)
%BUILDFEASIBLEDYNAMICINSTANCE 生成并尽量保存已知可行参考路线的动态实例
%   这是Benchmark构造工具，不是最终在线算法。

if nargin<4
    referenceBudget = 2400;
end

baseModel.cfg.silent = true;
instance = GenerateDynamicInstance(baseModel,scenarioSeed,level);
for e = 1:numel(instance.events)
    if strcmpi(instance.events(e).type,'add')
        for k = 1:numel(instance.events(e).customers)
            % 先给新增订单一个宽松窗口，先构造参考路线。
            instance.events(e).customers(k).window = [0 2000];
        end
    end
end

model = baseModel;
activeIDs = model.customerIDs(:)';
cache = BuildLegCache(model,activeIDs,model.homePosition);
rng(880000+scenarioSeed,'twister');
[solution,~,~] = RoutingPSO(model,cache,50,30, ...
    0.90,0.995,1.7,1.7,[]);

audit = repmat(struct('eventIndex',0,'status','UNKNOWN', ...
    'referenceCost',inf,'referenceFE',0,'tightened',false, ...
    'referenceRoute',[]),numel(instance.events),1);

for e = 1:numel(instance.events)
    event = instance.events(e);
    state = ExecuteUntilEvent(solution,model,event.time);
    activeIDs = setdiff(activeIDs,state.completedIDs,'stable');

    model = ApplyInstanceEvent(model,event);
    if strcmpi(event.type,'cancel')
        activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
    else
        activeIDs = [activeIDs,event.customerIDs]; %#ok<AGROW>
    end
    model.startTime = event.time;
    model.depot = state.position;
    [cache,~] = UpdateLegCache(model,cache,activeIDs,state.position);

    oracle = ExactTWFeasibilityOracle(model,cache,activeIDs,15);
    if strcmp(oracle.status,'FEASIBLE')
        reference.isFound = true;
        reference.referenceCost = oracle.objective;
        reference.totalFE = 0;
        reference.referenceSolution = SolutionFromRoute(oracle.referenceRoute,model,cache);
    else
        reference = BuildReferenceSolution(model,cache,activeIDs, ...
            referenceBudget,1);
    end
    audit(e).eventIndex = e;
    audit(e).status = ternary(reference.isFound,'FEASIBLE','UNKNOWN');
    audit(e).referenceCost = reference.referenceCost;
    audit(e).referenceFE = reference.totalFE;
    if ~reference.isFound
        break;
    end

    solution = reference.referenceSolution;
    if strcmpi(event.type,'add')
        slack = EventSlack(level);
        for k = 1:numel(event.customers)
            id = event.customers(k).id;
            hit = find(solution.Detail.records(:,1)==id,1);
            if ~isempty(hit)
                serviceStart = solution.Detail.records(hit,3);
                newWindow = [max(0,serviceStart-slack), ...
                    serviceStart+slack];
                instance.events(e).customers(k).window = newWindow;
                model.windows(id,:) = newWindow;
            end
        end
        [tightCost,tightDetail] = EvaluateSchedule( ...
            solution.Detail.routeIDs,model,cache);
        if tightDetail.isFeasible
            solution.Cost = tightCost;
            solution.Detail = tightDetail;
            solution.Route = tightDetail.routeIDs;
            audit(e).tightened = true;
        else
            % 若收紧后破坏参考可行性，恢复宽松窗口并保留UNKNOWN标记。
            for k = 1:numel(event.customers)
                id = event.customers(k).id;
                instance.events(e).customers(k).window = [0 2000];
                model.windows(id,:) = [0 2000];
            end
            audit(e).status = 'UNKNOWN_AFTER_TIGHTEN';
        end
    end
    audit(e).referenceRoute = solution.Detail.routeIDs;
end

instance.audit = audit;
instance.referenceModel = model;
end

function model = ApplyInstanceEvent(model,event)
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


function solution = SolutionFromRoute(routeIDs,model,cache)
[cost,detail] = EvaluateSchedule(routeIDs,model,cache);
solution.Cost = cost;
solution.Detail = detail;
solution.Route = detail.routeIDs;
end

function value = ternary(condition,a,b)
if condition
    value = a;
else
    value = b;
end
end


