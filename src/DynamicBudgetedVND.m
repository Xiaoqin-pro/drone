function [bestRoute,bestCost,bestDetail,stats] = DynamicBudgetedVND(route,model,cache,maxFE,initialDetail,options)
%DYNAMICBUDGETEDVND 动态三维评价下的FE-budgeted异构VND
if nargin<6 || isempty(options), options=struct(); end
if nargin<5 || isempty(initialDetail)
    [bestCost,bestDetail]=EvaluateSchedule(route,model,cache); functionEvaluations=1;
else
    bestDetail=initialDetail; bestCost=initialDetail.cost; functionEvaluations=0;
end
if nargin<4 || isempty(maxFE), maxFE=inf; end
if ~isfield(options,'neighborhoodQuota')||isempty(options.neighborhoodQuota)
    options.neighborhoodQuota=max(1,numel(cache.customerIDs)-1);
end
if ~isfield(options,'neighborhoodOrder')||isempty(options.neighborhoodOrder)
    options.neighborhoodOrder=["2opt","swap","relocate"];
end
bestRoute=route(:)'; k=1;
stats.functionEvaluations=functionEvaluations; stats.improvementCount=0;
stats.firstFeasibleEvaluation=inf;
stats.twoOptFE=0; stats.swapFE=0; stats.relocateFE=0;
stats.twoOptAccept=0; stats.swapAccept=0; stats.relocateAccept=0;
stats.neighborhoodVisits=zeros(numel(options.neighborhoodOrder),1);
while k<=numel(options.neighborhoodOrder) && functionEvaluations<maxFE
    neighborhood=string(options.neighborhoodOrder(k));
    candidates=BuildCandidates(bestRoute,neighborhood); order=randperm(numel(candidates));
    stats.neighborhoodVisits(k)=stats.neighborhoodVisits(k)+1;
    localRoute=bestRoute; localDetail=bestDetail; localCost=bestCost; improved=false;
    quota=min(options.neighborhoodQuota,maxFE-functionEvaluations);
    for q=order(1:min(numel(order),quota))
        if functionEvaluations>=maxFE, break; end
        [candidateCost,candidateDetail]=EvaluateSchedule(candidates{q},model,cache);
        functionEvaluations=functionEvaluations+1; stats=AddFE(stats,neighborhood);
        if isinf(stats.firstFeasibleEvaluation) && candidateDetail.isFeasible
            stats.firstFeasibleEvaluation=functionEvaluations;
        end
        if IsBetterDynamicSolution(candidateCost,candidateDetail,localCost,localDetail)
            localRoute=candidates{q}; localDetail=candidateDetail; localCost=candidateCost; improved=true;
        end
    end
    if improved
        bestRoute=localRoute; bestDetail=localDetail; bestCost=localCost;
        stats.improvementCount=stats.improvementCount+1; stats=AddAccept(stats,neighborhood); k=1;
    else
        k=k+1;
    end
end
stats.functionEvaluations=functionEvaluations;
end

function candidates=BuildCandidates(route,neighborhood)
route=route(:)'; n=numel(route); candidates=cell(0,1);
switch lower(char(neighborhood))
    case 'relocate'
        for i=1:n
            remaining=route([1:i-1,i+1:n]);
            for j=0:numel(remaining)
                candidate=[remaining(1:j),route(i),remaining(j+1:end)];
                if ~isequal(candidate,route), candidates{end+1,1}=candidate; end %#ok<AGROW>
            end
        end
    case 'swap'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate([i,j])=candidate([j,i]); candidates{end+1,1}=candidate; %#ok<AGROW>
            end
        end
    case '2opt'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate(i:j)=route(j:-1:i); candidates{end+1,1}=candidate; %#ok<AGROW>
            end
        end
end
end
function stats=AddFE(stats,neighborhood)
switch lower(char(neighborhood))
    case '2opt', stats.twoOptFE=stats.twoOptFE+1;
    case 'swap', stats.swapFE=stats.swapFE+1;
    case 'relocate', stats.relocateFE=stats.relocateFE+1;
end
end
function stats=AddAccept(stats,neighborhood)
switch lower(char(neighborhood))
    case '2opt', stats.twoOptAccept=stats.twoOptAccept+1;
    case 'swap', stats.swapAccept=stats.swapAccept+1;
    case 'relocate', stats.relocateAccept=stats.relocateAccept+1;
end
end
