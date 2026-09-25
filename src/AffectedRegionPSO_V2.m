function [BestSol,BestCost,stats,history] = AffectedRegionPSO_V2(model,cache,incumbentRoute,freeIDs,MaxIt,nPop,w,wdamp,c1,c2,maxFE)
%AFFECTEDREGIONPSO_V2 统一partial random-key编码的受影响区域PSO
if nargin<10 || isempty(maxFE), maxFE=nPop*(MaxIt+1); end
activeIDs=cache.customerIDs(:)'; incumbentRoute=incumbentRoute(:)'; freeIDs=intersect(freeIDs,activeIDs,'stable');
fixedIDs=activeIDs(~ismember(activeIDs,freeIDs)); nVar=numel(freeIDs);
VarMin=zeros(1,nVar); VarMax=ones(1,nVar); VelMax=0.20*ones(1,nVar); VelMin=-VelMax;
fixedKeys=FixedAnchors(incumbentRoute,fixedIDs); incumbentPosition=FreeKeys(incumbentRoute,freeIDs);
empty.Position=[]; empty.Velocity=[]; empty.Cost=[]; empty.Detail=[]; empty.Best.Position=[]; empty.Best.Cost=[]; empty.Best.Detail=[];
particles=repmat(empty,nPop,1); GlobalBest.Cost=inf; functionEvaluations=0; firstFeasible=inf;
routeKeys={}; previousKeys=repmat({''},nPop,1); routeChanges=0; improvementCount=0; lastImprovementFE=0;
[incCost,incDetail]=EvaluateSchedule(incumbentRoute,model,cache); functionEvaluations=1;
GlobalBest.Position=incumbentPosition; GlobalBest.Cost=incCost; GlobalBest.Detail=incDetail;
for i=1:nPop
    if i==1, pos=incumbentPosition; else, pos=rand(1,nVar); end
    particles(i).Position=pos; particles(i).Velocity=zeros(1,nVar);
    route=DecodePartial(pos,activeIDs,fixedIDs,fixedKeys,freeIDs); key=RouteKey(route); routeKeys{end+1,1}=key; %#ok<AGROW>
    if i>1 && ~strcmp(previousKeys{i},key), routeChanges=routeChanges+1; end
    previousKeys{i}=key;
    [particles(i).Cost,particles(i).Detail]=EvaluateSchedule(route,model,cache); functionEvaluations=functionEvaluations+1;
    particles(i).Best.Position=pos; particles(i).Best.Cost=particles(i).Cost; particles(i).Best.Detail=particles(i).Detail;
    if particles(i).Detail.isFeasible && isinf(firstFeasible), firstFeasible=functionEvaluations; end
    if particles(i).Cost<GlobalBest.Cost, GlobalBest=particles(i).Best; improvementCount=improvementCount+1; lastImprovementFE=functionEvaluations; end
end
BestCost=zeros(MaxIt,1); history.FE=zeros(MaxIt,1); history.Cost=zeros(MaxIt,1); history.IsFeasible=false(MaxIt,1);
for it=1:MaxIt
    if functionEvaluations+nPop>maxFE, break; end
    for i=1:nPop
        particles(i).Velocity=w*particles(i).Velocity+c1*rand(1,nVar).*(particles(i).Best.Position-particles(i).Position)+c2*rand(1,nVar).*(GlobalBest.Position-particles(i).Position);
        particles(i).Velocity=max(VelMin,min(VelMax,particles(i).Velocity)); particles(i).Position=max(VarMin,min(VarMax,particles(i).Position+particles(i).Velocity));
        route=DecodePartial(particles(i).Position,activeIDs,fixedIDs,fixedKeys,freeIDs); key=RouteKey(route); routeKeys{end+1,1}=key; %#ok<AGROW>
        if ~strcmp(previousKeys{i},key), routeChanges=routeChanges+1; end
        previousKeys{i}=key;
        [particles(i).Cost,particles(i).Detail]=EvaluateSchedule(route,model,cache); functionEvaluations=functionEvaluations+1;
        if particles(i).Detail.isFeasible && isinf(firstFeasible), firstFeasible=functionEvaluations; end
        if particles(i).Cost<particles(i).Best.Cost
            particles(i).Best.Position=particles(i).Position; particles(i).Best.Cost=particles(i).Cost; particles(i).Best.Detail=particles(i).Detail;
            if particles(i).Cost<GlobalBest.Cost, GlobalBest=particles(i).Best; improvementCount=improvementCount+1; lastImprovementFE=functionEvaluations; end
        end
    end
    BestCost(it)=GlobalBest.Cost; history.FE(it)=functionEvaluations; history.Cost(it)=GlobalBest.Cost; history.IsFeasible(it)=GlobalBest.Detail.isFeasible; w=w*wdamp;
end
BestSol=GlobalBest; BestSol.Route=BestSol.Detail.routeIDs;
stats.functionEvaluations=functionEvaluations; stats.firstFeasibleEvaluation=firstFeasible; stats.freeIDs=freeIDs; stats.fixedIDs=fixedIDs;
stats.uniqueRouteRatio=numel(unique(routeKeys))/max(numel(routeKeys),1); stats.routeChangeRate=routeChanges/max(functionEvaluations-nPop,1);
stats.improvementCount=improvementCount; stats.lastImprovementFE=lastImprovementFE;
end
function keys=FixedAnchors(route,ids)
keys=zeros(1,numel(ids)); n=numel(route); for k=1:numel(ids), pos=find(route==ids(k),1); if isempty(pos), keys(k)=rand; else, keys(k)=pos/(n+1); end, end
end
function keys=FreeKeys(route,ids)
keys=zeros(1,numel(ids)); n=numel(route); for k=1:numel(ids), pos=find(route==ids(k),1); if isempty(pos), keys(k)=rand; else, keys(k)=pos/(n+1); end, end
end
function route=DecodePartial(pos,activeIDs,fixedIDs,fixedKeys,freeIDs)
allIDs=[fixedIDs,freeIDs]; allKeys=[fixedKeys,pos]; [~,order]=sort(allKeys,'ascend'); route=allIDs(order);
end
function key=RouteKey(route)
key=sprintf('%d_',route);
end
