function [BestSol,BestCost,stats,history] = RoutingPSO(model,cache,MaxIt,nPop,w,wdamp,c1,c2,initialPositions,searchOptions)
%ROUTINGPSO 上层路由PSO：动态三维无人机的访问顺序优化
%   initialPositions为空时为Restart-PSO；非空时为Warm-start PSO。
%   searchOptions.enabled=true时，每轮在当前全局最优路线后执行有限FE的Relocate强化。

if nargin<9, initialPositions=[]; end
if nargin<10 || isempty(searchOptions), searchOptions=struct(); end
searchOptions=FillSearchOptions(searchOptions,cache);
runTimer=tic;

nVar=numel(cache.customerIDs);
VarMin=zeros(1,nVar); VarMax=ones(1,nVar);
VelMax=0.20*(VarMax-VarMin); VelMin=-VelMax;
empty.Position=[]; empty.Velocity=[]; empty.Cost=[]; empty.Detail=[];
empty.Best.Position=[]; empty.Best.Cost=[]; empty.Best.Detail=[];
particle=repmat(empty,nPop,1); GlobalBest.Cost=inf; GlobalBest.Detail=[]; GlobalBest.Position=[];
functionEvaluations=0; firstFeasibleIteration=inf; firstFeasibleEvaluation=inf;
boundaryReached=false; phaseSwitchFE=inf; phaseSwitchCost=NaN;

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
    if IsBetterDynamicSolution(particle(i).Cost,particle(i).Detail, ...
            GlobalBest.Cost,GlobalBest.Detail)
        GlobalBest=particle(i).Best;
    end
end
if GlobalBest.Detail.isFeasible
    firstFeasibleIteration=0; boundaryReached=true;
end

stats.initialBestCost=GlobalBest.Cost;
stats.initialBestFeasible=GlobalBest.Detail.isFeasible;
stats.initialLate=GlobalBest.Detail.totalLate;
stats.initialObstacleViolation=GlobalBest.Detail.totalObstacleViolation;
stats.initialTerrainViolation=GlobalBest.Detail.totalTerrainViolation;
stats.localSearchImprovementCount=0;
stats.acceptedSourceRanks=zeros(0,1);
stats.priorityFE=0;
stats.globalFE=0;
BestCost=zeros(MaxIt,1);
history.FE=zeros(MaxIt,1); history.Cost=zeros(MaxIt,1);
history.Distance=zeros(MaxIt,1); history.Late=zeros(MaxIt,1);
history.ObstacleViolation=zeros(MaxIt,1); history.TerrainViolation=zeros(MaxIt,1);
history.IsFeasible=false(MaxIt,1);
history.LocalSearchFE=zeros(MaxIt,1); history.Time=zeros(MaxIt,1);
history.RestorationFE=zeros(MaxIt,1); history.IntensificationFE=zeros(MaxIt,1);
history.BoundaryActive=false(MaxIt,1);
restorationFE=0; intensificationFE=0;
localSearchFE=0;
completedIterations=0;
for it=1:MaxIt
    if functionEvaluations>=searchOptions.maxFE, break; end
    for i=1:nPop
        if functionEvaluations>=searchOptions.maxFE, break; end
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
        if IsBetterDynamicSolution(particle(i).Cost,particle(i).Detail, ...
                particle(i).Best.Cost,particle(i).Best.Detail)
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            particle(i).Best.Detail=particle(i).Detail;
            if IsBetterDynamicSolution(particle(i).Best.Cost,particle(i).Best.Detail, ...
                    GlobalBest.Cost,GlobalBest.Detail)
                GlobalBest=particle(i).Best;
            end
        end
    end

    if ~boundaryReached && GlobalBest.Detail.isFeasible
        boundaryReached=true;
        if isinf(firstFeasibleEvaluation), firstFeasibleEvaluation=functionEvaluations; end
    end
    if searchOptions.enabled && searchOptions.localSearchFE>0 && functionEvaluations<searchOptions.maxFE
        mode=lower(string(searchOptions.mode));
        localBudget=searchOptions.localSearchFE;
        if mode=="boundary-vnd"
            if boundaryReached
                mode="dynamic-vnd"; localBudget=searchOptions.intensificationFE;
            else
                mode="late-relocate"; localBudget=searchOptions.restorationFE;
            end
        end
        if mode=="dynamic-vnd"
            if isinf(phaseSwitchFE)
                phaseSwitchFE=functionEvaluations; phaseSwitchCost=GlobalBest.Cost;
            end
            vndOptions=searchOptions; vndOptions.neighborhoodQuota=max(1,numel(cache.customerIDs)-1);
            [candidateRoute,candidateCost,candidateDetail,searchStats]=DynamicBudgetedVND( ...
                GlobalBest.Detail.routeIDs,model,cache,min(localBudget,searchOptions.maxFE-functionEvaluations), ...
                GlobalBest.Detail,vndOptions);
        else
            localOptions=searchOptions;
            if mode=="late-relocate"
                [priorityIDs,priorityScores]=BuildLatePriority(GlobalBest.Detail,cache.customerIDs);
                localOptions.impactIDs=priorityIDs;
                localOptions.impactScores=priorityScores;
                localOptions.mode='late-relocate';
            end
            [candidateRoute,candidateCost,candidateDetail,searchStats]= ...
                DynamicRelocateSearch(GlobalBest.Detail.routeIDs,model,cache, ...
                localOptions,min(localBudget,searchOptions.maxFE-functionEvaluations),GlobalBest.Detail);
        end
        if ~exist('candidateCost','var') || isempty(candidateCost), candidateCost=candidateDetail.cost; end
        localStartFE=functionEvaluations;
        functionEvaluations=functionEvaluations+searchStats.functionEvaluations;
        if isinf(firstFeasibleEvaluation) && isfield(searchStats,'firstFeasibleEvaluation') && isfinite(searchStats.firstFeasibleEvaluation)
            firstFeasibleEvaluation=localStartFE+searchStats.firstFeasibleEvaluation;
            firstFeasibleIteration=it; boundaryReached=true;
        end
        localSearchFE=localSearchFE+searchStats.functionEvaluations;
        if mode=="dynamic-vnd", intensificationFE=intensificationFE+searchStats.functionEvaluations;
        else, restorationFE=restorationFE+searchStats.functionEvaluations; end
        stats.localSearchImprovementCount=stats.localSearchImprovementCount+searchStats.improvementCount;
        if isfield(searchStats,'priorityFE'), stats.priorityFE=stats.priorityFE+searchStats.priorityFE; end
        if isfield(searchStats,'globalFE'), stats.globalFE=stats.globalFE+searchStats.globalFE; end
        if isfield(searchStats,'acceptedSourceRanks'), stats.acceptedSourceRanks=[stats.acceptedSourceRanks;searchStats.acceptedSourceRanks]; end
        if IsBetterDynamicSolution(candidateCost,candidateDetail, ...
                GlobalBest.Cost,GlobalBest.Detail)
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
    history.TerrainViolation(it)=GlobalBest.Detail.totalTerrainViolation;
    history.IsFeasible(it)=GlobalBest.Detail.isFeasible;
    history.LocalSearchFE(it)=localSearchFE;
    history.Time(it)=toc(runTimer);
    history.RestorationFE(it)=restorationFE;
    history.IntensificationFE(it)=intensificationFE;
    history.BoundaryActive(it)=boundaryReached;
    completedIterations=it;
    silent=isfield(model,'cfg') && isfield(model.cfg,'silent') && model.cfg.silent;
    if ~silent && (mod(it,10)==0 || it==1 || it==MaxIt)
        fprintf('Iteration %3d/%3d: distance=%.2f, delay=%.2f, ', ...
            it,MaxIt,GlobalBest.Detail.distance,GlobalBest.Detail.totalLate);
        fprintf('obstacle=%.2f, fitness=%.2f\n', ...
            GlobalBest.Detail.totalObstacleViolation,GlobalBest.Cost);
    end
    w=w*wdamp;
end

if completedIterations<MaxIt
    BestCost=BestCost(1:completedIterations);
    names=fieldnames(history);
    for h=1:numel(names)
        history.(names{h})=history.(names{h})(1:completedIterations,:);
    end
end
BestSol=GlobalBest;
BestSol.Route=BestSol.Detail.routeIDs;
stats.functionEvaluations=functionEvaluations;
stats.firstFeasibleIteration=firstFeasibleIteration;
stats.firstFeasibleEvaluation=firstFeasibleEvaluation;
stats.isWarmStart=~isempty(initialPositions);
stats.localSearchFE=localSearchFE;
stats.localSearchMode=string(searchOptions.mode);
stats.boundaryReached=boundaryReached; stats.phaseSwitchFE=phaseSwitchFE;
stats.phaseSwitchCost=phaseSwitchCost;
stats.restorationFE=restorationFE; stats.intensificationFE=intensificationFE;
end

function options=FillSearchOptions(options,cache)
defaults=struct('enabled',false,'mode','none','localSearchFE',0, ...
    'restorationFE',max(1,numel(cache.customerIDs)-1), ...
    'intensificationFE',3*max(1,numel(cache.customerIDs)-1),'maxFE',inf, ...
    'priorityIDs',cache.customerIDs(:)','impactIDs',cache.customerIDs(:)', ...
    'impactScores',zeros(size(cache.customerIDs(:)')),'impactAlpha',0.7);
fields=fieldnames(defaults);
for k=1:numel(fields)
    field=fields{k};
    if ~isfield(options,field) || isempty(options.(field)), options.(field)=defaults.(field); end
end
options.priorityIDs=intersect(options.priorityIDs,cache.customerIDs(:)','stable');
options.impactIDs=options.impactIDs(:)';
options.impactScores=options.impactScores(:)';
end

function [priorityIDs,priorityScores]=BuildLatePriority(detail,customerIDs)
priorityIDs=customerIDs(:)'; priorityScores=zeros(size(priorityIDs));
if isfield(detail,'records') && ~isempty(detail.records)
    late=zeros(size(priorityIDs));
    for k=1:size(detail.records,1)
        idx=find(priorityIDs==detail.records(k,1),1);
        if ~isempty(idx), late(idx)=max(0,detail.records(k,4)); end
    end
    if sum(late)<=0, late(:)=1; end
    priorityScores=late;
end
end

function position=RouteToPosition(route,customerIDs)
position=zeros(1,numel(customerIDs));
for k=1:numel(route)
    idx=find(customerIDs==route(k),1);
    if ~isempty(idx), position(idx)=k/max(numel(route),1); end
end
end

