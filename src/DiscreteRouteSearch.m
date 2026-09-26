function [bestRoute,bestDetail,stats] = DiscreteRouteSearch(initialRoute,instance,options,initialDetail)
%DISCRETEROUTESEARCH 使用离散邻域改善TSPTW路线
%   mode='relocate'时只使用Relocate；mode='full'时依次使用Relocate、Swap和2-opt。
%   maxFE严格限制候选路线评价次数，便于和PSO公平比较。

if nargin<3 || isempty(options)
    options = struct();
end
if nargin<4
    initialDetail = [];
end
if ~isfield(options,'mode') || isempty(options.mode)
    options.mode = 'relocate';
end
if ~isfield(options,'maxFE') || isempty(options.maxFE)
    options.maxFE = inf;
end
if ~isfield(options,'latePenalty') || isempty(options.latePenalty)
    options.latePenalty = 1000;
end
if ~isfield(options,'waitPenalty') || isempty(options.waitPenalty)
    options.waitPenalty = 0;
end

bestRoute = initialRoute(:)';
if isempty(initialDetail)
    [bestCost,bestDetail] = EvaluateTSPTWRoute(bestRoute,instance,options);
    functionEvaluations = 1;
else
    bestCost = initialDetail.cost;
    bestDetail = initialDetail;
    functionEvaluations = 0;
end

stats.functionEvaluations = functionEvaluations;
stats.improvementCount = 0;
stats.moves = strings(0,1);
if functionEvaluations>=options.maxFE
    return;
end

mode = lower(string(options.mode));
while functionEvaluations<options.maxFE
    passBestRoute = bestRoute;
    passBestDetail = bestDetail;
    passBestCost = bestCost;
    passMove = "";

    neighborhoodNames = "relocate";
    if mode=="full"
        neighborhoodNames = ["relocate","swap","2opt"];
    elseif mode~="relocate"
        error('不支持的离散搜索模式：%s。',options.mode);
    end

    for neighborhood = neighborhoodNames
        [candidates,moves] = BuildNeighborhood(bestRoute,neighborhood);
        for q = 1:numel(candidates)
            if functionEvaluations>=options.maxFE
                break;
            end
            [candidateCost,candidateDetail] = ...
                EvaluateTSPTWRoute(candidates{q},instance,options);
            functionEvaluations = functionEvaluations+1;
            if candidateCost<passBestCost-1e-12
                passBestRoute = candidates{q};
                passBestDetail = candidateDetail;
                passBestCost = candidateCost;
                passMove = moves(q);
            end
        end
        if functionEvaluations>=options.maxFE
            break;
        end
    end

    if passBestCost<bestCost-1e-12
        bestRoute = passBestRoute;
        bestDetail = passBestDetail;
        bestCost = passBestCost;
        stats.improvementCount = stats.improvementCount+1;
        stats.moves(end+1,1) = passMove; %#ok<AGROW>
    else
        break;
    end
end

stats.functionEvaluations = functionEvaluations;
end

function [candidates,moves] = BuildNeighborhood(route,neighborhood)
route = route(:)';
n = numel(route);
candidates = cell(0,1);
moves = strings(0,1);

switch lower(char(neighborhood))
    case 'relocate'
        for i = 1:n
            remaining = route([1:i-1,i+1:n]);
            for j = 0:numel(remaining)
                candidate = [remaining(1:j),route(i),remaining(j+1:end)];
                if isequal(candidate,route), continue; end
                candidates{end+1,1} = candidate; %#ok<AGROW>
                moves(end+1,1) = "relocate("+i+","+j+")"; %#ok<AGROW>
            end
        end
    case 'swap'
        for i = 1:n-1
            for j = i+1:n
                candidate = route;
                candidate([i,j]) = candidate([j,i]);
                candidates{end+1,1} = candidate; %#ok<AGROW>
                moves(end+1,1) = "swap("+i+","+j+")"; %#ok<AGROW>
            end
        end
    case '2opt'
        for i = 1:n-1
            for j = i+1:n
                candidate = route;
                candidate(i:j) = route(j:-1:i);
                candidates{end+1,1} = candidate; %#ok<AGROW>
                moves(end+1,1) = "2opt("+i+","+j+")"; %#ok<AGROW>
            end
        end
    otherwise
        error('未知邻域：%s',neighborhood);
end
end
