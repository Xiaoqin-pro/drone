function [bestRoute,bestCost,bestDetail,stats] = RouteLocalSearch( ...
    route,model,cache,affectedIDs,maxMoves,maxEvaluations)
%ROUTELOCALSEARCH 对受影响区域执行Relocate、Swap和2-opt邻域搜索
if nargin<5 || isempty(maxMoves), maxMoves=30; end
if nargin<6 || isempty(maxEvaluations), maxEvaluations=inf; end
route=route(:)'; affectedIDs=affectedIDs(:)';
[bestCost,bestDetail]=EvaluateSchedule(route,model,cache);
bestRoute=route; functionEvaluations=1;
for move=1:maxMoves
    improved=false; candidateRoutes={};
    for id=affectedIDs
        p=find(bestRoute==id,1); if isempty(p), continue; end
        for q=1:numel(bestRoute)
            if q==p, continue; end
            candidate=bestRoute; candidate(p)=[];
            if q>numel(candidate), candidate=[candidate,id];
            else, candidate=[candidate(1:q-1),id,candidate(q:end)]; end
            candidateRoutes{end+1}=candidate; %#ok<AGROW>
        end
    end
    for id=affectedIDs
        p=find(bestRoute==id,1); if isempty(p), continue; end
        for q=1:numel(bestRoute)
            if q==p, continue; end
            candidate=bestRoute; candidate([p q])=candidate([q p]);
            candidateRoutes{end+1}=candidate; %#ok<AGROW>
        end
    end
    for p=1:numel(bestRoute)-1
        for q=p+1:numel(bestRoute)
            if ~ismember(bestRoute(p),affectedIDs) && ~ismember(bestRoute(q),affectedIDs), continue; end
            candidate=[bestRoute(1:p-1),fliplr(bestRoute(p:q)),bestRoute(q+1:end)];
            candidateRoutes{end+1}=candidate; %#ok<AGROW>
        end
    end
    if isempty(candidateRoutes), break; end
    localBestCost=bestCost; localBestRoute=bestRoute; localBestDetail=bestDetail;
    for k=randperm(numel(candidateRoutes))
        [cost,detail]=EvaluateSchedule(candidateRoutes{k},model,cache);
        functionEvaluations=functionEvaluations+1;
        if cost<localBestCost
            localBestCost=cost; localBestRoute=candidateRoutes{k}; localBestDetail=detail;
        end
    end
    if localBestCost<bestCost
        bestCost=localBestCost; bestRoute=localBestRoute; bestDetail=localBestDetail; improved=true;
    end
    if ~improved, break; end
end
stats.functionEvaluations=functionEvaluations; stats.moves=move;
end
