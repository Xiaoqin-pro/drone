function [bestRoute,bestDetail,stats,history] = GVNSTSPTW(instance,options)
%GVNSTSPTW 面向TSPTW的通用变邻域搜索基线
%   这是仓库内的可复现实验基线：随机扰动(shaking)+BudgetedVND局部强化。
%   所有路线评价严格计入maxFE；比较规则与PSO一致：先totalLate，再tourCost。

if nargin<2 || isempty(options), options=struct(); end
options=FillOptions(options,instance);
n=instance.nCustomers;
route=InitialRoute(instance,options);
[bestCost,bestDetail]=EvaluateTSPTWRoute(route,instance,options);
functionEvaluations=options.initialEvaluations;
if functionEvaluations==0, functionEvaluations=1; end
firstFeasibleFE=inf;
if bestDetail.isFeasible, firstFeasibleFE=functionEvaluations; end
bestRoute=route; iteration=0; noImprovement=0; restartCount=0;
history.FE=zeros(0,1); history.Cost=zeros(0,1); history.TourCost=zeros(0,1);
history.Late=zeros(0,1); history.IsFeasible=false(0,1); history.Neighborhood=zeros(0,1);
while functionEvaluations<options.maxFE
    iteration=iteration+1;
    improved=false;
    for k=1:options.maxNeighborhood
        if functionEvaluations>=options.maxFE, break; end
        [shaken,shakeFE]=ShakeRoute(bestRoute,k,instance,options);
        functionEvaluations=functionEvaluations+shakeFE;
        remaining=options.maxFE-functionEvaluations;
        if remaining<=0, break; end
        localOptions=options;
        localOptions.neighborhoodQuota=max(1,n-1);
        localOptions.neighborhoodOrder=options.neighborhoodOrder;
        localBudget=min(options.localSearchFE,remaining);
        localStartFE=functionEvaluations;
        [candidateRoute,candidateDetail,localStats]=BudgetedVND( ...
            shaken,instance,localBudget,[],localOptions);
        functionEvaluations=functionEvaluations+localStats.functionEvaluations;
        if isinf(firstFeasibleFE) && isfinite(localStats.firstFeasibleEvaluation)
            firstFeasibleFE=localStartFE+localStats.firstFeasibleEvaluation;
        end
        if IsBetter(candidateDetail.cost,candidateDetail,bestCost,bestDetail)
            bestRoute=candidateRoute; bestDetail=candidateDetail; bestCost=candidateDetail.cost;
            improved=true; noImprovement=0;
            history.Neighborhood(end+1,1)=k;
            break;
        end
        if functionEvaluations>=options.maxFE, break; end
    end
    if ~improved, noImprovement=noImprovement+1; end
    history.FE(end+1,1)=functionEvaluations;
    history.Cost(end+1,1)=bestCost;
    history.TourCost(end+1,1)=bestDetail.tourCost;
    history.Late(end+1,1)=bestDetail.totalLate;
    history.IsFeasible(end+1,1)=bestDetail.isFeasible;
    if numel(history.Neighborhood)<numel(history.FE), history.Neighborhood(end+1,1)=0; end
    if noImprovement>=options.maxNoImprovement
        noImprovement=0;
        restartCount=restartCount+1;
    end
end
BestSol.Route=bestRoute; BestSol.Cost=bestCost; BestSol.Detail=bestDetail;
stats.functionEvaluations=functionEvaluations;
stats.firstFeasibleFE=firstFeasibleFE;
stats.iterations=iteration;
stats.initialEvaluations=options.initialEvaluations;
stats.isFeasible=bestDetail.isFeasible;
stats.restartCount=restartCount;
end

function route=InitialRoute(instance,options)
if isfield(options,'initialRoute') && ~isempty(options.initialRoute)
    route=options.initialRoute(:)'; return;
end
% 结构化但不使用BKS的初始化：先按due time/ready time排序，再做随机扰动。
if isfield(options,'initialMode') && string(options.initialMode)=="random"
    route=instance.customerIDs(randperm(instance.nCustomers));
else
    ids=instance.customerIDs(:);
    key=[instance.windows(ids,2),instance.windows(ids,1),rand(numel(ids),1)];
    [~,order]=sortrows(key,[1 2 3]); route=ids(order)';
end
end

function [route,fe] = ShakeRoute(route,k,instance,options)
route=route(:)'; fe=0; n=numel(route);
for move=1:k
    if n<2, break; end
    i=randi(n); j=randi(n);
    while j==i, j=randi(n); end
    if rand<0.5
        route([i,j])=route([j,i]);
    else
        node=route(i); route(i)=[];
        if j>numel(route)+1, j=numel(route)+1; end
        route=[route(1:j-1),node,route(j:end)];
    end
end
% Shaking only changes a permutation; no objective evaluation is performed here.
fe=0;
end

function options=FillOptions(options,instance)
defaults=struct('maxFE',1000,'maxNeighborhood',3,'localSearchFE', ...
    max(1,instance.nCustomers-1),'neighborhoodQuota',max(1,instance.nCustomers-1), ...
    'neighborhoodOrder',["2opt","swap","relocate"],'maxNoImprovement',10, ...
    'latePenalty',1000,'waitPenalty',0,'initialEvaluations',1,'initialMode','earliest_due','silent',true);
fields=fieldnames(defaults);
for k=1:numel(fields)
    f=fields{k}; if ~isfield(options,f)||isempty(options.(f)), options.(f)=defaults.(f); end
end
options.maxFE=max(1,round(options.maxFE));
options.localSearchFE=max(1,round(options.localSearchFE));
options.maxNeighborhood=max(1,round(options.maxNeighborhood));
end

function tf=IsBetter(candidateCost,candidateDetail,bestCost,bestDetail)
if candidateDetail.totalLate<bestDetail.totalLate-1e-10
    tf=true;
elseif abs(candidateDetail.totalLate-bestDetail.totalLate)<=1e-10
    tf=candidateDetail.tourCost<bestDetail.tourCost-1e-10;
else
    tf=false;
end
end

