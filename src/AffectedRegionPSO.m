function [BestSol,BestCost,stats,history] = AffectedRegionPSO( ...
    model,cache,previousRoute,affectedIDs,MaxIt,nPop,w,wdamp,c1,c2,useLocalSearch,maxFE)
%AFFECTEDREGIONPSO PSO只重点搜索动态事件影响区域
%   非受影响客户的相对顺序保持，受影响客户及其邻域参与搜索。
if nargin<10, useLocalSearch=false; end
if nargin<11 || isempty(maxFE), maxFE=nPop*(MaxIt+1); end
activeIDs=cache.customerIDs(:)';
previousRoute=previousRoute(:)';
affectedIDs=affectedIDs(:)';
affectedIDs=intersect(affectedIDs,activeIDs,'stable');
fixedRoute=previousRoute(ismember(previousRoute,activeIDs) & ...
    ~ismember(previousRoute,affectedIDs));
missingFixed = activeIDs(~ismember(activeIDs,[fixedRoute affectedIDs]));
fixedRoute=[fixedRoute,missingFixed];
nVar=numel(activeIDs);
mask=double(ismember(activeIDs,affectedIDs));
VarMin=zeros(1,nVar); VarMax=ones(1,nVar);
VelMax=0.20*(VarMax-VarMin); VelMin=-VelMax;
empty.Position=[]; empty.Velocity=[]; empty.Cost=[]; empty.Detail=[];
empty.Best.Position=[]; empty.Best.Cost=[]; empty.Best.Detail=[];
particle=repmat(empty,nPop,1); GlobalBest.Cost=inf;
functionEvaluations=0; firstFeasibleEvaluation=inf;

for i=1:nPop
    particle(i).Position=rand(1,nVar);
    particle(i).Velocity=zeros(1,nVar);
    route=DecodeAffectedRoute(particle(i).Position,activeIDs,fixedRoute,affectedIDs);
    [particle(i).Cost,particle(i).Detail]=EvaluateSchedule(route,model,cache);
    functionEvaluations=functionEvaluations+1;
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    particle(i).Best.Detail=particle(i).Detail;
    if particle(i).Detail.isFeasible && isinf(firstFeasibleEvaluation)
        firstFeasibleEvaluation=functionEvaluations;
    end
    if particle(i).Cost<GlobalBest.Cost, GlobalBest=particle(i).Best; end
end

BestCost=zeros(MaxIt,1);
history.FE=zeros(MaxIt,1); history.Cost=zeros(MaxIt,1);
history.Distance=zeros(MaxIt,1); history.Late=zeros(MaxIt,1);
history.IsFeasible=false(MaxIt,1);
for it=1:MaxIt
    if functionEvaluations+nPop>maxFE, break; end
    for i=1:nPop
        particle(i).Velocity=particle(i).Velocity.*(1-mask) ...
            + (w*particle(i).Velocity ...
            + c1*rand(1,nVar).*(particle(i).Best.Position-particle(i).Position) ...
            + c2*rand(1,nVar).*(GlobalBest.Position-particle(i).Position)).*mask;
        particle(i).Velocity=max(VelMin,min(VelMax,particle(i).Velocity));
        particle(i).Position=particle(i).Position+particle(i).Velocity;
        particle(i).Position=max(VarMin,min(VarMax,particle(i).Position));
        route=DecodeAffectedRoute(particle(i).Position,activeIDs,fixedRoute,affectedIDs);
        [particle(i).Cost,particle(i).Detail]=EvaluateSchedule(route,model,cache);
        functionEvaluations=functionEvaluations+1;
        if particle(i).Detail.isFeasible && isinf(firstFeasibleEvaluation)
            firstFeasibleEvaluation=functionEvaluations;
        end
        if particle(i).Cost<particle(i).Best.Cost
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            particle(i).Best.Detail=particle(i).Detail;
            if particle(i).Cost<GlobalBest.Cost, GlobalBest=particle(i).Best; end
        end
    end

    if useLocalSearch
        remainingFE=maxFE-functionEvaluations;
[route,cost,detail,ls]=RouteLocalSearch(GlobalBest.Detail.routeIDs, ...
            model,cache,affectedIDs,2,min(50,remainingFE));
        functionEvaluations=functionEvaluations+ls.functionEvaluations;
        if cost<GlobalBest.Cost
            GlobalBest.Position=GlobalBest.Position;
            GlobalBest.Cost=cost;
            GlobalBest.Detail=detail;
            GlobalBest.Route=route;
        end
    end
    BestCost(it)=GlobalBest.Cost;
    history.FE(it)=functionEvaluations;
    history.Cost(it)=GlobalBest.Cost;
    history.Distance(it)=GlobalBest.Detail.distance;
    history.Late(it)=GlobalBest.Detail.totalLate;
    history.IsFeasible(it)=GlobalBest.Detail.isFeasible;
    w=w*wdamp;
end
BestSol=GlobalBest;
BestSol.Route=BestSol.Detail.routeIDs;
if isfield(GlobalBest,'Route'), BestSol.Route=GlobalBest.Route; end
stats.functionEvaluations=functionEvaluations;
stats.firstFeasibleEvaluation=firstFeasibleEvaluation;
stats.affectedIDs=affectedIDs;
stats.fixedCount=numel(fixedRoute);
stats.affectedCount=numel(affectedIDs);
end

function route=DecodeAffectedRoute(position,activeIDs,fixedRoute,affectedIDs)
affectedOrder=affectedIDs;
keys=position(ismember(activeIDs,affectedIDs));
[~,order]=sort(keys,'ascend');
affectedOrder=affectedOrder(order);
route=fixedRoute;
for id=affectedOrder
    key=position(activeIDs==id);
    pos=1+round(key*(numel(route)));
    pos=min(max(pos,1),numel(route)+1);
    route=[route(1:pos-1),id,route(pos:end)]; %#ok<AGROW>
end
end
