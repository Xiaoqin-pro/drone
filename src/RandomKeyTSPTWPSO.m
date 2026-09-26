function [BestSol,history,stats] = RandomKeyTSPTWPSO(instance,options,initialPositions)
%RANDOMKEYTSPTWPSO Random-key PSO基线及离散邻域增强版本
%   localSearchMode='none'     ：纯Random-key PSO
%   localSearchMode='relocate' ：PSO每代后追加Relocate搜索
%   localSearchMode='full'     ：PSO每代后追加Relocate、Swap、2-opt搜索
%
%   所有路线评价均计入maxFE，三种方法可以在相同FE预算下比较。

if nargin<2 || isempty(options)
    options = struct();
end
if nargin<3
    initialPositions = [];
end

options = FillOptions(options,instance);
nVar = instance.nCustomers;
nPop = min(options.nPop,max(1,floor(options.maxFE)));
if nPop<2
    nPop = 1;
end

VarMin = zeros(1,nVar);
VarMax = ones(1,nVar);
VelMax = options.velocityRatio*(VarMax-VarMin);
VelMin = -VelMax;

empty.Position = [];
empty.Velocity = [];
empty.Cost = inf;
empty.Detail = [];
empty.Best.Position = [];
empty.Best.Cost = inf;
empty.Best.Detail = [];
particle = repmat(empty,nPop,1);
GlobalBest.Cost = inf;
GlobalBest.Position = [];
GlobalBest.Detail = [];
functionEvaluations = 0;

for i = 1:nPop
    if ~isempty(initialPositions) && i<=size(initialPositions,1)
        particle(i).Position = initialPositions(i,:);
    else
        particle(i).Position = rand(1,nVar);
    end
    particle(i).Position = max(VarMin,min(VarMax,particle(i).Position));
    particle(i).Velocity = zeros(1,nVar);
    route = DecodeRoute(particle(i).Position,instance.customerIDs);
    [particle(i).Cost,particle(i).Detail] = ...
        EvaluateTSPTWRoute(route,instance,options);
    functionEvaluations = functionEvaluations+1;
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
    particle(i).Best.Detail = particle(i).Detail;
    if particle(i).Cost<GlobalBest.Cost
        GlobalBest = particle(i).Best;
    end
end

history.FE = zeros(0,1);
history.Cost = zeros(0,1);
history.Distance = zeros(0,1);
history.Late = zeros(0,1);
history.IsFeasible = false(0,1);
history.LocalSearchFE = zeros(0,1);
localSearchFE = 0;
iteration = 0;

while functionEvaluations<options.maxFE
    iteration = iteration+1;
    for i = 1:nPop
        if functionEvaluations>=options.maxFE
            break;
        end
        particle(i).Velocity = options.w*particle(i).Velocity ...
            + options.c1*rand(1,nVar).*(particle(i).Best.Position ...
            -particle(i).Position) ...
            + options.c2*rand(1,nVar).*(GlobalBest.Position ...
            -particle(i).Position);
        particle(i).Velocity = max(VelMin,min(VelMax,particle(i).Velocity));
        particle(i).Position = particle(i).Position+particle(i).Velocity;
        particle(i).Position = max(VarMin,min(VarMax,particle(i).Position));

        route = DecodeRoute(particle(i).Position,instance.customerIDs);
        [particle(i).Cost,particle(i).Detail] = ...
            EvaluateTSPTWRoute(route,instance,options);
        functionEvaluations = functionEvaluations+1;
        if particle(i).Cost<particle(i).Best.Cost
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            particle(i).Best.Detail = particle(i).Detail;
            if particle(i).Best.Cost<GlobalBest.Cost
                GlobalBest = particle(i).Best;
            end
        end
    end

    if options.localSearchFE>0 && functionEvaluations<options.maxFE ...
            && mod(iteration,options.localSearchEvery)==0
        remaining = options.maxFE-functionEvaluations;
        budget = min(options.localSearchFE,remaining);
        searchOptions = options;
        searchOptions.mode = options.localSearchMode;
        searchOptions.maxFE = budget;
        [candidateRoute,candidateDetail,searchStats] = ...
            DiscreteRouteSearch(GlobalBest.Detail.route,instance, ...
            searchOptions,GlobalBest.Detail);
        functionEvaluations = functionEvaluations+searchStats.functionEvaluations;
        localSearchFE = localSearchFE+searchStats.functionEvaluations;
        if candidateDetail.cost<GlobalBest.Cost
            GlobalBest.Route = candidateRoute;
            GlobalBest.Detail = candidateDetail;
            GlobalBest.Cost = candidateDetail.cost;
            GlobalBest.Position = RouteToKeys(candidateRoute,instance.customerIDs);
        end
    end

    history.FE(end+1,1) = functionEvaluations;
    history.Cost(end+1,1) = GlobalBest.Cost;
    history.Distance(end+1,1) = GlobalBest.Detail.distance;
    history.Late(end+1,1) = GlobalBest.Detail.totalLate;
    history.IsFeasible(end+1,1) = GlobalBest.Detail.isFeasible;
    history.LocalSearchFE(end+1,1) = localSearchFE;
    options.w = options.w*options.wdamp;
end

BestSol = GlobalBest;
BestSol.Route = GlobalBest.Detail.route;
stats.functionEvaluations = functionEvaluations;
stats.iterations = iteration;
stats.localSearchFE = localSearchFE;
stats.localSearchMode = string(options.localSearchMode);
stats.initialPopulation = nPop;
end

function options = FillOptions(options,instance)
defaults = struct('nPop',20,'maxFE',1000,'w',1.0,'wdamp',0.99, ...
    'c1',1.5,'c2',1.5,'velocityRatio',0.20,'latePenalty',1000, ...
    'waitPenalty',0,'localSearchMode','none','localSearchFE',0, ...
    'localSearchEvery',1,'silent',true);
fields = fieldnames(defaults);
for k = 1:numel(fields)
    field = fields{k};
    if ~isfield(options,field) || isempty(options.(field))
        options.(field) = defaults.(field);
    end
end
options.nPop = max(1,round(options.nPop));
options.maxFE = max(1,round(options.maxFE));
options.localSearchEvery = max(1,round(options.localSearchEvery));
if numel(instance.customerIDs)~=size(instance.windows,1)-1
    error('TSPTW实例客户节点和时间窗数量不一致。');
end
end

function position = RouteToKeys(route,customerIDs)
% 将离散路线映射回一组严格递增的random-key，便于记录而非参与评价。
position = zeros(1,numel(customerIDs));
for k = 1:numel(route)
    idx = find(customerIDs==route(k),1);
    position(idx) = k/numel(route);
end
end

