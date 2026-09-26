function [BestSol,history,stats] = RandomKeyTSPTWPSO(instance,options,initialPositions)
%RANDOMKEYTSPTWPSO Random-key PSO基线及离散邻域增强版本
%   localSearchMode='none'     ：纯Random-key PSO
%   localSearchMode='relocate' ：PSO每代后追加Relocate搜索
%   localSearchMode='swap'     ：PSO每代后追加Swap搜索
%   localSearchMode='2opt'     ：PSO每代后追加2-opt搜索
%   localSearchMode='mixed'    ：三种离散算子公平混合采样
%
%   适应度更新采用可行性优先规则：先比较totalLate，再比较tourCost。
%   所有路线评价均计入maxFE，便于公平比较。

if nargin<2 || isempty(options), options = struct(); end
if nargin<3, initialPositions = []; end
options = FillOptions(options,instance);
nVar = instance.nCustomers;
nPop = min(options.nPop,max(1,floor(options.maxFE)));

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
feedbackDirection=zeros(1,nVar); feedbackUpdates=0;
firstFeasibleFE = inf;
firstFeasibleRoute = []; firstFeasibleDetail = [];

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
    if isinf(firstFeasibleFE) && particle(i).Detail.isFeasible
        firstFeasibleFE = functionEvaluations;
        firstFeasibleRoute = route; firstFeasibleDetail = particle(i).Detail;
    end
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
    particle(i).Best.Detail = particle(i).Detail;
    if IsBetterSolution(particle(i).Cost,particle(i).Detail, ...
            GlobalBest.Cost,GlobalBest.Detail)
        GlobalBest = particle(i).Best;
    end
end

history.FE = zeros(0,1);
history.Cost = zeros(0,1);
history.TourCost = zeros(0,1);
history.Distance = zeros(0,1);
history.Late = zeros(0,1);
history.IsFeasible = false(0,1);
history.LocalSearchFE = zeros(0,1);
history.LocalSearchRelocateFE = zeros(0,1);
history.LocalSearchSwapFE = zeros(0,1);
history.LocalSearchTwoOptFE = zeros(0,1);
history.Mode = strings(0,1);
history.StateSwitch = false(0,1);
history.FeedbackNorm = zeros(0,1); history.FeedbackUpdates = zeros(0,1);
localSearchFE = 0;
localSearchRelocateFE = 0;
localSearchSwapFE = 0;
localSearchTwoOptFE = 0;
iteration = 0; lastMode = "";

while functionEvaluations<options.maxFE
    iteration = iteration+1;
    for i = 1:nPop
        if functionEvaluations>=options.maxFE, break; end
        particle(i).Velocity = options.w*particle(i).Velocity ...
            + options.c1*rand(1,nVar).*(particle(i).Best.Position ...
            -particle(i).Position) ...
            + options.c2*rand(1,nVar).*(GlobalBest.Position ...
            -particle(i).Position);
        if options.feedbackEnabled
            particle(i).Velocity=particle(i).Velocity+ ...
                options.feedbackLearningRate*feedbackDirection;
        end
        particle(i).Velocity = max(VelMin,min(VelMax,particle(i).Velocity));
        particle(i).Position = particle(i).Position+particle(i).Velocity;
        particle(i).Position = max(VarMin,min(VarMax,particle(i).Position));
        route = DecodeRoute(particle(i).Position,instance.customerIDs);
        [particle(i).Cost,particle(i).Detail] = ...
            EvaluateTSPTWRoute(route,instance,options);
        functionEvaluations = functionEvaluations+1;
        if isinf(firstFeasibleFE) && particle(i).Detail.isFeasible
            firstFeasibleFE = functionEvaluations;
            firstFeasibleRoute = route; firstFeasibleDetail = particle(i).Detail;
        end
        if IsBetterSolution(particle(i).Cost,particle(i).Detail, ...
                particle(i).Best.Cost,particle(i).Best.Detail)
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            particle(i).Best.Detail = particle(i).Detail;
            if IsBetterSolution(particle(i).Best.Cost,particle(i).Best.Detail, ...
                    GlobalBest.Cost,GlobalBest.Detail)
                GlobalBest = particle(i).Best;
            end
        end
    end

    if options.localSearchFE>0 && functionEvaluations<options.maxFE ...
            && mod(iteration,options.localSearchEvery)==0
        remaining = options.maxFE-functionEvaluations;
        searchOptions = options;
        searchMode = string(options.localSearchMode);
        if searchMode=="state-switch"
            if GlobalBest.Detail.isFeasible, searchMode="global"; else, searchMode="propagation"; end
        end
        stateSwitch = lastMode~="" && searchMode~=lastMode;
        lastMode = searchMode;
        searchOptions.mode = char(searchMode);
        searchOptions.maxFE = min(options.localSearchFE,remaining);
        routeBeforeLocal=GlobalBest.Detail.route;
        [candidateRoute,candidateDetail,searchStats] = ...
            DiscreteRouteSearch(routeBeforeLocal,instance, ...
            searchOptions,GlobalBest.Detail);
        functionEvaluations = functionEvaluations+searchStats.functionEvaluations;
        localSearchFE = localSearchFE+searchStats.functionEvaluations;
        localSearchRelocateFE = localSearchRelocateFE+searchStats.relocateFE;
        localSearchSwapFE = localSearchSwapFE+searchStats.swapFE;
        localSearchTwoOptFE = localSearchTwoOptFE+searchStats.twoOptFE;
        if isinf(firstFeasibleFE) && isfinite(searchStats.firstFeasibleEvaluation)
            firstFeasibleFE = functionEvaluations-searchStats.functionEvaluations ...
                +searchStats.firstFeasibleEvaluation;
            if ~isempty(searchStats.firstFeasibleRoute)
                firstFeasibleRoute = searchStats.firstFeasibleRoute;
                firstFeasibleDetail = searchStats.firstFeasibleDetail;
            else
                firstFeasibleRoute = candidateRoute; firstFeasibleDetail = candidateDetail;
            end
        end
        if IsBetterSolution(candidateDetail.cost,candidateDetail, ...
                GlobalBest.Cost,GlobalBest.Detail)
            GlobalBest.Route = candidateRoute;
            GlobalBest.Detail = candidateDetail;
            GlobalBest.Cost = candidateDetail.cost;
            GlobalBest.Position = RouteToKeys(candidateRoute,instance.customerIDs);
            if options.feedbackEnabled
                [newDirection,feedbackDetail]=BuildRelocateLearningDirection( ...
                    routeBeforeLocal, candidateRoute, instance.customerIDs); %#ok<ASGLU>
                feedbackDirection=options.feedbackDecay*feedbackDirection ...
                    +(1-options.feedbackDecay)*newDirection;
                feedbackUpdates=feedbackUpdates+1;
            end
            if candidateDetail.isFeasible && isinf(firstFeasibleFE)
                firstFeasibleFE = functionEvaluations;
            end
        end
    end

    history.FE(end+1,1) = functionEvaluations;
    history.Cost(end+1,1) = GlobalBest.Cost;
    history.TourCost(end+1,1) = GlobalBest.Detail.tourCost;
    history.Distance(end+1,1) = GlobalBest.Detail.tourCost;
    history.Late(end+1,1) = GlobalBest.Detail.totalLate;
    history.IsFeasible(end+1,1) = GlobalBest.Detail.isFeasible;
    history.LocalSearchFE(end+1,1) = localSearchFE;
    history.LocalSearchRelocateFE(end+1,1) = localSearchRelocateFE;
    history.LocalSearchSwapFE(end+1,1) = localSearchSwapFE;
    history.LocalSearchTwoOptFE(end+1,1) = localSearchTwoOptFE;
    if exist('searchMode','var'), history.Mode(end+1,1)=searchMode; else, history.Mode(end+1,1)=string(options.localSearchMode); end
    if exist('stateSwitch','var'), history.StateSwitch(end+1,1)=stateSwitch; else, history.StateSwitch(end+1,1)=false; end
    history.FeedbackNorm(end+1,1)=norm(feedbackDirection);
    history.FeedbackUpdates(end+1,1)=feedbackUpdates;
    options.w = options.w*options.wdamp;
end

BestSol = GlobalBest;
BestSol.Route = GlobalBest.Detail.route;
BestSol.TourCost = GlobalBest.Detail.tourCost;
stats.functionEvaluations = functionEvaluations;
stats.iterations = iteration;
stats.firstFeasibleFE = firstFeasibleFE;
stats.firstFeasibleRoute = firstFeasibleRoute;
stats.firstFeasibleDetail = firstFeasibleDetail;
stats.localSearchFE = localSearchFE;
stats.localSearchRelocateFE = localSearchRelocateFE;
stats.localSearchSwapFE = localSearchSwapFE;
stats.localSearchTwoOptFE = localSearchTwoOptFE;
stats.localSearchMode = string(options.localSearchMode);
stats.feedbackUpdates=feedbackUpdates;
stats.feedbackDirection=feedbackDirection;
stats.initialPopulation = nPop;
end

function options = FillOptions(options,instance)
defaults = struct('nPop',20,'maxFE',1000,'w',1.0,'wdamp',0.99, ...
    'c1',1.5,'c2',1.5,'velocityRatio',0.20,'latePenalty',1000, ...
    'waitPenalty',0,'localSearchMode','none','localSearchFE',0, ...
    'localSearchEvery',1,'silent',true, ...
    'feedbackEnabled',false,'feedbackLearningRate',0.35,'feedbackDecay',0.80);
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
position = zeros(1,numel(customerIDs));
for k = 1:numel(route)
    idx = find(customerIDs==route(k),1);
    position(idx) = k/numel(route);
end
end

function tf = IsBetterSolution(candidateCost,candidateDetail,bestCost,bestDetail)
if isempty(bestDetail)
    tf = true;
    return;
end
if candidateDetail.totalLate<bestDetail.totalLate-1e-10
    tf = true;
elseif abs(candidateDetail.totalLate-bestDetail.totalLate)<=1e-10
    tf = candidateDetail.tourCost<bestDetail.tourCost-1e-10;
else
    tf = false;
end
end
