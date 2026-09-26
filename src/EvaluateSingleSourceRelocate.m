function [bestRoute,bestCost,bestDetail,stats] = EvaluateSingleSourceRelocate( ...
    route,instance,sourceNode,initialDetail,options)
%EVALUATESINGLESOURCERELocate 完整评价一个source customer的所有reinsertion位置
%   该函数把一个source neighborhood作为原子搜索单元，避免不同规模下FE含义不一致。

if nargin<5 || isempty(options), options=struct(); end
if nargin<4 || isempty(initialDetail)
    [bestCost,bestDetail]=EvaluateTSPTWRoute(route,instance,options);
    initialEvaluations=1;
else
    bestDetail=initialDetail; bestCost=initialDetail.cost; initialEvaluations=0;
end
route=route(:)';
sourcePosition=find(route==sourceNode,1);
if isempty(sourcePosition), error('sourceNode不在当前路线中。'); end
remaining=route; remaining(sourcePosition)=[];
positionOrder=1:(numel(remaining)+1);
functionEvaluations=initialEvaluations;
bestRoute=route; improvementCount=0; firstFeasibleEvaluation=inf;
for position=positionOrder
    candidate=[remaining(1:position-1),sourceNode,remaining(position:end)];
    if isequal(candidate,route), continue; end
    [candidateCost,candidateDetail]=EvaluateTSPTWRoute(candidate,instance,options);
    functionEvaluations=functionEvaluations+1;
    if isinf(firstFeasibleEvaluation) && candidateDetail.isFeasible
        firstFeasibleEvaluation=functionEvaluations;
    end
    if IsBetterTSPTWSolution(candidateCost,candidateDetail,bestCost,bestDetail)
        bestRoute=candidate; bestCost=candidateCost; bestDetail=candidateDetail;
        improvementCount=improvementCount+1;
    end
end
stats.functionEvaluations=functionEvaluations;
stats.improvementCount=improvementCount;
stats.sourceNode=sourceNode;
stats.firstFeasibleEvaluation=firstFeasibleEvaluation;
stats.isFeasible=bestDetail.isFeasible;
stats.lateReduction=max(0,initialDetailOrValue(route,instance,initialDetail,options,'late')-bestDetail.totalLate);
end

function value=initialDetailOrValue(route,instance,detail,options,name)
if ~isempty(detail)
    value=detail.totalLate;
else
    [~,d]=EvaluateTSPTWRoute(route,instance,options); value=d.(name);
end
end

function tf=IsBetterTSPTWSolution(candidateCost,candidateDetail,bestCost,bestDetail)
if isempty(bestDetail), tf=true; return; end
if candidateDetail.totalLate<bestDetail.totalLate-1e-10
    tf=true;
elseif abs(candidateDetail.totalLate-bestDetail.totalLate)<=1e-10
    tf=candidateDetail.tourCost<bestDetail.tourCost-1e-10;
else
    tf=false;
end
end
