function [bestRoute,bestDetail,stats] = BudgetedVND(route,instance,maxFE,initialDetail,options)
%BUDGETEDVND 有限FE下的Variable Neighborhood Descent
%   默认顺序：2-opt -> Swap -> Relocate。
%   找到改善后回到第一个邻域；一个邻域无改善才进入下一个邻域。

if nargin<5 || isempty(options), options=struct(); end
if nargin<4 || isempty(initialDetail)
    [bestCost,bestDetail]=EvaluateTSPTWRoute(route,instance,options);
    functionEvaluations=1;
else
    bestDetail=initialDetail; bestCost=initialDetail.cost; functionEvaluations=0;
end
if nargin<3 || isempty(maxFE), maxFE=inf; end
if ~isfield(options,'neighborhoodOrder') || isempty(options.neighborhoodOrder)
    neighborhoodOrder=["2opt","swap","relocate"];
else
    neighborhoodOrder=string(options.neighborhoodOrder);
end
bestRoute=route(:)'; k=1; improvementCount=0; moves=strings(0,1);
while k<=numel(neighborhoodOrder) && functionEvaluations<maxFE
    [candidates,moveNames]=BuildCandidates(bestRoute,neighborhoodOrder(k));
    order=randperm(numel(candidates)); improved=false;
    localRoute=bestRoute; localDetail=bestDetail; localCost=bestCost; localMove="";
    for q=order
        if functionEvaluations>=maxFE, break; end
        [candidateCost,candidateDetail]=EvaluateTSPTWRoute(candidates{q},instance,options);
        functionEvaluations=functionEvaluations+1;
        if IsBetter(candidateCost,candidateDetail,localCost,localDetail)
            localRoute=candidates{q}; localDetail=candidateDetail;
            localCost=candidateCost; localMove=moveNames(q); improved=true;
        end
    end
    if improved
        bestRoute=localRoute; bestDetail=localDetail; bestCost=localCost;
        improvementCount=improvementCount+1; moves(end+1,1)=localMove; %#ok<AGROW>
        k=1;
    else
        k=k+1;
    end
end
stats.functionEvaluations=functionEvaluations;
stats.improvementCount=improvementCount;
stats.moves=moves;
stats.neighborhoodOrder=neighborhoodOrder;
end

function [candidates,moves]=BuildCandidates(route,neighborhood)
route=route(:)'; n=numel(route); candidates=cell(0,1); moves=strings(0,1);
switch lower(char(neighborhood))
    case 'relocate'
        for i=1:n
            remaining=route([1:i-1,i+1:n]);
            for j=0:numel(remaining)
                candidate=[remaining(1:j),route(i),remaining(j+1:end)];
                if isequal(candidate,route), continue; end
                candidates{end+1,1}=candidate; %#ok<AGROW>
                moves(end+1,1)="relocate("+i+","+j+")"; %#ok<AGROW>
            end
        end
    case 'swap'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate([i,j])=candidate([j,i]);
                candidates{end+1,1}=candidate; %#ok<AGROW>
                moves(end+1,1)="swap("+i+","+j+")"; %#ok<AGROW>
            end
        end
    case '2opt'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate(i:j)=route(j:-1:i);
                candidates{end+1,1}=candidate; %#ok<AGROW>
                moves(end+1,1)="2opt("+i+","+j+")"; %#ok<AGROW>
            end
        end
    otherwise
        error('未知VND邻域：%s',neighborhood);
end
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
