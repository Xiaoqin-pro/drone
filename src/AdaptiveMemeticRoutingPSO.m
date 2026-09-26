function [BestSol,history,stats] = AdaptiveMemeticRoutingPSO(model,cache,options,initialPositions)
%ADAPTIVEMEMETICROUTINGPSO 可行性状态与FE收益自适应的模因PSO
%   不可行状态：按时间窗传播优先级执行Feasibility-Restoration Relocate；
%   可行状态：执行Quality-Intensification Relocate；
%   局部预算：依据归一化relative improvement per FE动态分配。

if nargin<2 || isempty(cache), error('需要提供路段缓存cache。'); end
if nargin<3 || isempty(options), options=struct(); end
if nargin<4, initialPositions=[]; end
options=FillOptions(options);
nVar=numel(cache.customerIDs);
nPop=min(options.nPop,max(1,floor(options.maxFE)));
VarMin=zeros(1,nVar); VarMax=ones(1,nVar);
VelMax=options.velocityRatio*(VarMax-VarMin); VelMin=-VelMax;
empty.Position=[]; empty.Velocity=[]; empty.Cost=inf; empty.Detail=[];
empty.Best.Position=[]; empty.Best.Cost=inf; empty.Best.Detail=[];
particles=repmat(empty,nPop,1);
GlobalBest.Cost=inf; GlobalBest.Detail=[]; GlobalBest.Position=[];
functionEvaluations=0; firstFeasibleEvaluation=inf;

for i=1:nPop
    if ~isempty(initialPositions) && i<=size(initialPositions,1)
        particles(i).Position=initialPositions(i,:);
    else
        particles(i).Position=rand(1,nVar);
    end
    particles(i).Position=max(VarMin,min(VarMax,particles(i).Position));
    particles(i).Velocity=zeros(1,nVar);
    route=DecodeRoute(particles(i).Position,cache.customerIDs);
    [particles(i).Cost,particles(i).Detail]=EvaluateSchedule(route,model,cache);
    functionEvaluations=functionEvaluations+1;
    if particles(i).Detail.isFeasible && isinf(firstFeasibleEvaluation)
        firstFeasibleEvaluation=functionEvaluations;
    end
    particles(i).Best.Position=particles(i).Position;
    particles(i).Best.Cost=particles(i).Cost;
    particles(i).Best.Detail=particles(i).Detail;
    if IsBetterDynamicSolution(particles(i).Cost,particles(i).Detail, ...
            GlobalBest.Cost,GlobalBest.Detail)
        GlobalBest=particles(i).Best;
    end
end

history.FE=zeros(0,1); history.Cost=zeros(0,1);
history.Distance=zeros(0,1); history.Late=zeros(0,1);
history.ObstacleViolation=zeros(0,1); history.IsFeasible=false(0,1);
history.FeasibilityViolation=zeros(0,1); history.LocalSearchFE=zeros(0,1);
history.LocalBudget=zeros(0,1); history.PSOImprovementPerFE=zeros(0,1);
history.LocalImprovementPerFE=zeros(0,1); history.EtaPSO=zeros(0,1);
history.EtaLocal=zeros(0,1); history.Mode=strings(0,1);
history.StateSwitch=false(0,1);
etaPSO=options.initialEfficiency; etaLocal=options.initialEfficiency;
localSearchFE=0; localSearchImprovementCount=0;
feasibilityModeFE=0; qualityModeFE=0; iteration=0;
acceptedSourceRanks=zeros(0,1); lastMode="";

while functionEvaluations<options.maxFE
    iteration=iteration+1;
    mode=CurrentMode(GlobalBest.Detail);
    stateSwitch=lastMode~="" && mode~=lastMode;
    if stateSwitch
        etaPSO=options.initialEfficiency;
        etaLocal=options.initialEfficiency;
    end
    lastMode=mode;
    beforePSODetail=GlobalBest.Detail;
    psoEvaluations=0;
    for i=1:nPop
        if functionEvaluations>=options.maxFE, break; end
        particles(i).Velocity=options.w*particles(i).Velocity ...
            +options.c1*rand(1,nVar).*(particles(i).Best.Position-particles(i).Position) ...
            +options.c2*rand(1,nVar).*(GlobalBest.Position-particles(i).Position);
        particles(i).Velocity=max(VelMin,min(VelMax,particles(i).Velocity));
        particles(i).Position=max(VarMin,min(VarMax,particles(i).Position+particles(i).Velocity));
        route=DecodeRoute(particles(i).Position,cache.customerIDs);
        [particles(i).Cost,particles(i).Detail]=EvaluateSchedule(route,model,cache);
        functionEvaluations=functionEvaluations+1; psoEvaluations=psoEvaluations+1;
        if particles(i).Detail.isFeasible && isinf(firstFeasibleEvaluation)
            firstFeasibleEvaluation=functionEvaluations;
        end
        if IsBetterDynamicSolution(particles(i).Cost,particles(i).Detail, ...
                particles(i).Best.Cost,particles(i).Best.Detail)
            particles(i).Best.Position=particles(i).Position;
            particles(i).Best.Cost=particles(i).Cost;
            particles(i).Best.Detail=particles(i).Detail;
            if IsBetterDynamicSolution(particles(i).Best.Cost,particles(i).Best.Detail, ...
                    GlobalBest.Cost,GlobalBest.Detail)
                GlobalBest=particles(i).Best;
            end
        end
    end
    psoGain=RelativeGain(beforePSODetail,GlobalBest.Detail,mode,model,psoEvaluations);
    etaPSO=(1-options.efficiencySmoothing)*etaPSO+options.efficiencySmoothing*psoGain;

    mode=CurrentMode(GlobalBest.Detail);
    remaining=options.maxFE-functionEvaluations;
    localBudget=AllocateLocalBudget(mode,etaPSO,etaLocal,options,remaining);
    localGain=0; localEvaluations=0;
    if localBudget>0 && remaining>0
        searchOptions=struct('mode',char(mode),'localSearchFE',localBudget, ...
            'impactAlpha',options.feasibilityAlpha,'impactIDs',cache.customerIDs(:)', ...
            'impactScores',zeros(1,nVar));
        if mode=="feasibility-relocate"
            [impactIDs,impactScores]=BuildFeasibilityPriority( ...
                GlobalBest.Detail,cache.customerIDs);
            searchOptions.impactIDs=impactIDs;
            searchOptions.impactScores=impactScores;
        end
        beforeLocalDetail=GlobalBest.Detail;
        [candidateRoute,candidateCost,candidateDetail,localStats]= ...
            DynamicRelocateSearch(GlobalBest.Detail.routeIDs,model,cache, ...
            searchOptions,localBudget,GlobalBest.Detail);
        localEvaluations=localStats.functionEvaluations;
        functionEvaluations=functionEvaluations+localEvaluations;
        localSearchFE=localSearchFE+localEvaluations;
        localSearchImprovementCount=localSearchImprovementCount+localStats.improvementCount;
        acceptedSourceRanks=[acceptedSourceRanks;localStats.acceptedSourceRanks]; %#ok<AGROW>
        if mode=="feasibility-relocate"
            feasibilityModeFE=feasibilityModeFE+localEvaluations;
        else
            qualityModeFE=qualityModeFE+localEvaluations;
        end
        if IsBetterDynamicSolution(candidateCost,candidateDetail, ...
                GlobalBest.Cost,GlobalBest.Detail)
            GlobalBest.Route=candidateRoute;
            GlobalBest.Cost=candidateCost;
            GlobalBest.Detail=candidateDetail;
            GlobalBest.Position=RouteToPosition(candidateRoute,cache.customerIDs);
            if candidateDetail.isFeasible && isinf(firstFeasibleEvaluation)
                firstFeasibleEvaluation=functionEvaluations;
            end
        end
        localGain=RelativeGain(beforeLocalDetail,GlobalBest.Detail, ...
            mode,model,localEvaluations);
        etaLocal=(1-options.efficiencySmoothing)*etaLocal ...
            +options.efficiencySmoothing*localGain;
    end

    history.FE(end+1,1)=functionEvaluations;
    history.Cost(end+1,1)=GlobalBest.Cost;
    history.Distance(end+1,1)=GlobalBest.Detail.distance;
    history.Late(end+1,1)=GlobalBest.Detail.totalLate;
    history.ObstacleViolation(end+1,1)=GlobalBest.Detail.totalObstacleViolation;
    history.IsFeasible(end+1,1)=GlobalBest.Detail.isFeasible;
    history.FeasibilityViolation(end+1,1)=ConstraintViolation(GlobalBest.Detail,model);
    history.LocalSearchFE(end+1,1)=localSearchFE;
    history.LocalBudget(end+1,1)=localBudget;
    history.PSOImprovementPerFE(end+1,1)=psoGain;
    history.LocalImprovementPerFE(end+1,1)=localGain;
    history.EtaPSO(end+1,1)=etaPSO; history.EtaLocal(end+1,1)=etaLocal;
    history.Mode(end+1,1)=mode; history.StateSwitch(end+1,1)=stateSwitch;
    options.w=options.w*options.wdamp;
end

BestSol=GlobalBest; BestSol.Route=GlobalBest.Detail.routeIDs;
stats.functionEvaluations=functionEvaluations;
stats.firstFeasibleEvaluation=firstFeasibleEvaluation;
stats.iterations=iteration; stats.localSearchFE=localSearchFE;
stats.localSearchImprovementCount=localSearchImprovementCount;
stats.feasibilityModeFE=feasibilityModeFE; stats.qualityModeFE=qualityModeFE;
stats.meanLocalBudget=mean(history.LocalBudget);
stats.acceptedSourceRanks=acceptedSourceRanks;
stats.finalEtaPSO=etaPSO; stats.finalEtaLocal=etaLocal;
stats.stateSwitchCount=sum(history.StateSwitch);
stats.isWarmStart=~isempty(initialPositions);
end

function options=FillOptions(options)
defaults=struct('nPop',30,'maxFE',2500,'w',0.90,'wdamp',0.995, ...
    'c1',1.7,'c2',1.7,'velocityRatio',0.20,'localMinFE',5,'localMaxFE',40, ...
    'efficiencySmoothing',0.25,'initialEfficiency',1.0,'feasibilityAlpha',0.85);
fields=fieldnames(defaults);
for k=1:numel(fields)
    field=fields{k};
    if ~isfield(options,field) || isempty(options.(field)), options.(field)=defaults.(field); end
end
options.nPop=max(1,round(options.nPop)); options.maxFE=max(1,round(options.maxFE));
options.localMinFE=max(0,round(options.localMinFE));
options.localMaxFE=max(options.localMinFE,round(options.localMaxFE));
end

function mode=CurrentMode(detail)
if detail.isFeasible, mode="quality-relocate"; else, mode="feasibility-relocate"; end
end

function budget=AllocateLocalBudget(mode,etaPSO,etaLocal,options,remaining)
if mode=="feasibility-relocate"
    target=options.localMaxFE;
else
    ratio=(etaLocal+eps)/(etaPSO+etaLocal+2*eps);
    target=round(options.localMinFE+(options.localMaxFE-options.localMinFE)*ratio);
end
budget=min(max(target,0),remaining);
end

function gain=RelativeGain(beforeDetail,afterDetail,mode,model,functionEvaluations)
if functionEvaluations<=0, gain=0; return; end
if mode=="feasibility-relocate"
    before=ConstraintViolation(beforeDetail,model);
    after=ConstraintViolation(afterDetail,model);
    gain=max(0,(before-after)/(before+eps))/functionEvaluations;
else
    if beforeDetail.isFeasible && afterDetail.isFeasible
        gain=max(0,(beforeDetail.cost-afterDetail.cost)/ ...
            (abs(beforeDetail.cost)+eps))/functionEvaluations;
    else
        before=ConstraintViolation(beforeDetail,model);
        after=ConstraintViolation(afterDetail,model);
        gain=max(0,(before-after)/(before+eps))/functionEvaluations;
    end
end
end

function value=ConstraintViolation(detail,model)
lateScale=max(1,sum(max(model.windows(:,2)-model.windows(:,1),0)));
obstacleScale=max(1,model.nObstacles*max(model.obstacleSafety,1));
terrainScale=max(1,model.nCustomers*max(model.minClearance,1));
value=detail.totalLate/lateScale ...
    +detail.totalObstacleViolation/obstacleScale ...
    +detail.totalTerrainViolation/terrainScale;
end

function [impactIDs,impactScores]=BuildFeasibilityPriority(detail,customerIDs)
impactScores=zeros(size(customerIDs(:)'));
if isfield(detail,'records') && ~isempty(detail.records)
    records=detail.records; n=size(records,1); suffix=zeros(n,1);
    for k=n:-1:1
        suffix(k)=records(k,4);
        if k<n, suffix(k)=suffix(k)+suffix(k+1); end
    end
    for k=1:n
        idx=find(customerIDs==records(k,1),1);
        if ~isempty(idx)
            impactScores(idx)=max(0,records(k,4))+0.5*suffix(k)/max(n-k+1,1);
        end
    end
end
if sum(impactScores)<=0, impactScores(:)=1; end
impactIDs=customerIDs(:)';
end

function position=RouteToPosition(route,customerIDs)
position=zeros(1,numel(customerIDs));
for k=1:numel(route)
    idx=find(customerIDs==route(k),1);
    if ~isempty(idx), position(idx)=k/max(numel(route),1); end
end
end
