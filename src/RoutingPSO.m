function [BestSol,BestCost,stats,history] = RoutingPSO(model,cache,MaxIt,nPop,w,wdamp,c1,c2,initialPositions,searchOptions)
%ROUTINGPSO 上层路由PSO：动态三维无人机的访问顺序优化
%   initialPositions为空时为Restart-PSO；非空时为Warm-start PSO。
%   searchOptions.enabled=true时，每轮在当前全局最优路线后执行有限FE的Relocate强化。

if nargin<9, initialPositions=[]; end
if nargin<10 || isempty(searchOptions), searchOptions=struct(); end
searchOptions=FillSearchOptions(searchOptions,cache);

nVar=numel(cache.customerIDs);
VarMin=zeros(1,nVar); VarMax=ones(1,nVar);
VelMax=0.20*(VarMax-VarMin); VelMin=-VelMax;
empty.Position=[]; empty.Velocity=[]; empty.Cost=[]; empty.Detail=[];
empty.Best.Position=[]; empty.Best.Cost=[]; empty.Best.Detail=[];
particle=repmat(empty,nPop,1); GlobalBest.Cost=inf;
functionEvaluations=0; firstFeasibleIteration=inf; firstFeasibleEvaluation=inf;

for i=1:nPop
    if isempty(initialPositions)
        particle(i).Position=rand(1,nVar);
    elseif i<=size(initialPositions,1)
        particle(i).Position=initialPositions(i,:);
    else
        particle(i).Position=rand(1,nVar);
    end
    particle(i).Position=max(VarMin,min(VarMax,particle(i).Position));
    particle(i).Velocity=zeros(1,nVar);
    routeIDs=DecodeRoute(particle(i).Position,cache.customerIDs);
    [particle(i).Cost,particle(i).Detail]=EvaluateSchedule(routeIDs,model,cache);
    functionEvaluations=functionEvaluations+1;
    if isinf(firstFeasibleEvaluation) && particle(i).Detail.isFeasible
        firstFeasibleEvaluation=functionEvaluations;
    end
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    particle(i).Best.Detail=particle(i).Detail;
    if particle(i).Cost<GlobalBest.Cost
        GlobalBest=particle(i).Best;
    end
end
if GlobalBest.Detail.isFeasible, firstFeasibleIteration=0; end

stats.initialBestCost=GlobalBest.Cost;
BestCost=zeros(MaxIt,1);
history.FE=zeros(MaxIt,1); history.Cost=zeros(MaxIt,1);
history.Distance=zeros(MaxIt,1); history.Late=zeros(MaxIt,1);
history.ObstacleViolation=zeros(MaxIt,1); history.IsFeasible=false(MaxIt,1);
history.LocalSearchFE=zeros(MaxIt,1);
localSearchFE=0;
for it=1:MaxIt
    for i=1:nPop
        particle(i).Velocity=w*particle(i).Velocity ...
            +c1*rand(1,nVar).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(1,nVar).*(GlobalBest.Position-particle(i).Position);
        particle(i).Velocity=max(VelMin,min(VelMax,particle(i).Velocity));
        particle(i).Position=max(VarMin,min(VarMax,particle(i).Position+particle(i).Velocity));
        routeIDs=DecodeRoute(particle(i).Position,cache.customerIDs);
        [particle(i).Cost,particle(i).Detail]=EvaluateSchedule(routeIDs,model,cache);
        functionEvaluations=functionEvaluations+1;
        if isinf(firstFeasibleEvaluation) && particle(i).Detail.isFeasible
            firstFeasibleEvaluation=functionEvaluations;
            firstFeasibleIteration=it;
        end
        if particle(i).Cost<particle(i).Best.Cost
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            particle(i).Best.Detail=particle(i).Detail;
            if particle(i).Best.Cost<GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end
        end
    end

    if searchOptions.enabled && searchOptions.localSearchFE>0
        [candidateRoute,candidateCost,candidateDetail,searchStats] = ...
            DynamicRelocateSearch(GlobalBest.Detail.routeIDs,model,cache, ...
            searchOptions.priorityIDs,searchOptions.localSearchFE,GlobalBest.Detail);
        functionEvaluations=functionEvaluations+searchStats.functionEvaluations;
        localSearchFE=localSearchFE+searchStats.functionEvaluations;
        if candidateCost<GlobalBest.Cost
            GlobalBest.Route=candidateRoute;
            GlobalBest.Cost=candidateCost;
            GlobalBest.Detail=candidateDetail;
            GlobalBest.Position=RouteToPosition(candidateRoute,cache.customerIDs);
            if candidateDetail.isFeasible && isinf(firstFeasibleEvaluation)
                firstFeasibleEvaluation=functionEvaluations;
                firstFeasibleIteration=it;
            end
        end
    end

    if isinf(firstFeasibleIteration) && GlobalBest.Detail.isFeasible
        firstFeasibleIteration=it;
    end
    BestCost(it)=GlobalBest.Cost;
    history.FE(it)=functionEvaluations;
    history.Cost(it)=GlobalBest.Cost;
    history.Distance(it)=GlobalBest.Detail.distance;
    history.Late(it)=GlobalBest.Detail.totalLate;
    history.ObstacleViolation(it)=GlobalBest.Detail.totalObstacleViolation;
    history.IsFeasible(it)=GlobalBest.Detail.isFeasible;
    history.LocalSearchFE(it)=localSearchFE;
    silent=isfield(model,'cfg') && isfield(model.cfg,'silent') && model.cfg.silent;
    if ~silent && (mod(it,10)==0 || it==1 || it==MaxIt)
        fprintf('Iteration %3d/%3d: distance=%.2f, delay=%.2f, ', ...
            it,MaxIt,GlobalBest.Detail.distance,GlobalBest.Detail.totalLate);
        fprintf('obstacle=%.2f, fitness=%.2f\n', ...
            GlobalBest.Detail.totalObstacleViolation,GlobalBest.Cost);
    end
    w=w*wdamp;
end

BestSol=GlobalBest;
BestSol.Route=BestSol.Detail.routeIDs;
stats.functionEvaluations=functionEvaluations;
stats.firstFeasibleIteration=firstFeasibleIteration;
stats.firstFeasibleEvaluation=firstFeasibleEvaluation;
stats.isWarmStart=~isempty(initialPositions);
stats.localSearchFE=localSearchFE;
stats.localSearchMode=string(searchOptions.mode);
end

function options=FillSearchOptions(options,cache)
defaults=struct('enabled',false,'mode','none','localSearchFE',0, ...
    'priorityIDs',cache.customerIDs(:)');
fields=fieldnames(defaults);
for k=1:numel(fields)
    field=fields{k};
    if ~isfield(options,field) || isempty(options.(field)), options.(field)=defaults.(field); end
end
options.priorityIDs=intersect(options.priorityIDs,cache.customerIDs(:)','stable');
end

function position=RouteToPosition(route,customerIDs)
position=zeros(1,numel(customerIDs));
for k=1:numel(route)
    idx=find(customerIDs==route(k),1);
    if ~isempty(idx), position(idx)=k/max(numel(route),1); end
end
end
