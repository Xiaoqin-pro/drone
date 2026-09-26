function [bestRoute,bestCost,bestDetail,stats] = DynamicRelocateSearch( ...
    initialRoute,model,cache,priorityIDs,maxFE,initialDetail)
%DYNAMICRELOCATESEARCH 动态三维航段评价下的有限FE Relocate强化
%   priorityIDs只控制候选源客户的优先顺序，不冻结其余客户。

if nargin<4 || isempty(priorityIDs), priorityIDs=cache.customerIDs(:)'; end
if nargin<5 || isempty(maxFE), maxFE=inf; end
if nargin<6 || isempty(initialDetail)
    [bestCost,bestDetail]=EvaluateSchedule(initialRoute,model,cache);
    functionEvaluations=1;
else
    bestDetail=initialDetail;
    if isfield(initialDetail,'cost')
        bestCost=initialDetail.cost;
        functionEvaluations=0;
    else
        [bestCost,bestDetail]=EvaluateSchedule(initialRoute,model,cache);
        functionEvaluations=1;
    end
end
bestRoute=initialRoute(:)';
priorityIDs=intersect(priorityIDs,bestRoute,'stable');
remainingIDs=setdiff(bestRoute,priorityIDs,'stable');
sourceIDs=[priorityIDs,remainingIDs(randperm(numel(remainingIDs)))];
stats.functionEvaluations=functionEvaluations;
stats.improvementCount=0;
stats.priorityFE=0;
stats.globalFE=0;
if functionEvaluations>=maxFE, return; end

candidateRoutes=cell(0,1);
candidatePriority=false(0,1);
for sourceIndex=1:numel(sourceIDs)
    id=sourceIDs(sourceIndex);
    isPriority=sourceIndex<=numel(priorityIDs);
    p=find(bestRoute==id,1);
    routeWithout=bestRoute; routeWithout(p)=[];
    for q=0:numel(routeWithout)
        candidate=[routeWithout(1:q),id,routeWithout(q+1:end)];
        if isequal(candidate,bestRoute), continue; end
        candidateRoutes{end+1,1}=candidate; %#ok<AGROW>
        candidatePriority(end+1,1)=isPriority; %#ok<AGROW>
    end
end
order=randperm(numel(candidateRoutes));
for orderIndex=1:numel(order)
    k=order(orderIndex);
    if functionEvaluations>=maxFE, break; end
    [candidateCost,candidateDetail]=EvaluateSchedule(candidateRoutes{k},model,cache);
    functionEvaluations=functionEvaluations+1;
    if candidatePriority(k)
        stats.priorityFE=stats.priorityFE+1;
    else
        stats.globalFE=stats.globalFE+1;
    end
    if candidateCost<bestCost
        bestRoute=candidateRoutes{k};
        bestCost=candidateCost;
        bestDetail=candidateDetail;
        stats.improvementCount=stats.improvementCount+1;
    end
end
stats.functionEvaluations=functionEvaluations;
end

