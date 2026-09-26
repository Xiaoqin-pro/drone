function [solution,info] = LocalRepair(previousSolution,activeIDs,model,cache,changeType,changedIDs)
%LOCALREPAIR 动态任务变化后的最小范围路线修复基线
%   取消：从旧路线删除无效任务；
%   新增：按时间窗感知的最优插入逐个加入路线。

activeIDs = activeIDs(:)';
changedIDs = changedIDs(:)';
route = previousSolution.Detail.routeIDs(:)';
route = route(ismember(route,activeIDs));
functionEvaluations = 0;

switch lower(changeType)
    case 'cancel'
        route = route(~ismember(route,changedIDs));
        [cost,detail] = EvaluateSchedule(route,model,cache);
        functionEvaluations = functionEvaluations+1;

    case 'add'
        for newID = changedIDs
            if ismember(newID,route)
                continue;
            end
            bestRoute = [];
            bestCost = inf;
            bestDetail = [];
            candidateCount = numel(route)+1;
            for position = 1:candidateCount
                candidateRoute = [route(1:position-1),newID, ...
                    route(position:end)];
                [candidateCost,candidateDetail] = ...
                    EvaluateSchedule(candidateRoute,model,cache);
                functionEvaluations = functionEvaluations+1;
                if IsBetterDynamicSolution(candidateCost,candidateDetail, ...
                        bestCost,bestDetail)
                    bestRoute = candidateRoute;
                    bestCost = candidateCost;
                    bestDetail = candidateDetail;
                end
            end
            route = bestRoute;
        end
        [cost,detail] = EvaluateSchedule(route,model,cache);
        functionEvaluations = functionEvaluations+1;

    otherwise
        error('Unknown repair type: %s',changeType);
end

solution.Cost = cost;
solution.Detail = detail;
solution.Route = detail.routeIDs;
info.functionEvaluations = functionEvaluations;
info.isFeasible = detail.isFeasible;
info.route = route;
end

