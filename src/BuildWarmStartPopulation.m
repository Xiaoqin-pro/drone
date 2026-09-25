function [initialPositions,info] = BuildWarmStartPopulation(previousSolution, ...
    activeIDs,model,cache,nPop)
%BUILDWARMSTARTPOPULATION 根据旧路线构造动态事件后的初始种群
%   取消任务：删除旧路线中的失效客户；
%   新增任务：尝试插入旧路线的不同位置；
%   其余粒子用局部交换/反转维持多样性。

activeIDs = activeIDs(:)';
oldRoute = previousSolution.Detail.routeIDs(:)';
baseRoute = oldRoute(ismember(oldRoute,activeIDs));
missingIDs = activeIDs(~ismember(activeIDs,baseRoute));

candidateRoutes = cell(0,1);
candidateRoutes{end+1} = baseRoute; %#ok<AGROW>

% 对新增客户尝试所有插入位置。
for m = 1:numel(missingIDs)
    newID = missingIDs(m);
    currentCandidates = candidateRoutes;
    candidateRoutes = cell(0,1);
    for c = 1:numel(currentCandidates)
        route = currentCandidates{c};
        for pos = 1:numel(route)+1
            candidateRoutes{end+1} = InsertAt(route,newID,pos); %#ok<AGROW>
        end
    end
end

% 用局部扰动补充候选解，避免warm start失去多样性。
for k = 1:max(2*nPop,20)
    if isempty(baseRoute)
        candidateRoutes{end+1} = activeIDs(randperm(numel(activeIDs))); %#ok<AGROW>
    else
        route = baseRoute;
        if ~isempty(missingIDs)
            route = candidateRoutes{randi(numel(candidateRoutes))};
        end
        if numel(route)>=2
            pair = randperm(numel(route),2);
            route(pair) = route(fliplr(pair));
        end
        candidateRoutes{end+1} = route; %#ok<AGROW>
    end
end

% 去除重复路线。
keys = cellfun(@RouteKey,candidateRoutes,'UniformOutput',false);
[~,uniqueIndex] = unique(keys,'stable');
candidateRoutes = candidateRoutes(uniqueIndex);

% 用当前缓存评价候选路线并选出前nPop个。
% 限制候选评价数量，避免多个新增订单导致组合爆炸。
maxCandidateEvaluations = max(5*nPop,200);
if numel(candidateRoutes)>maxCandidateEvaluations
    keepIndex = randperm(numel(candidateRoutes),maxCandidateEvaluations);
    candidateRoutes = candidateRoutes(keepIndex);
end
costs = inf(numel(candidateRoutes),1);
for k = 1:numel(candidateRoutes)
    [costs(k),~] = EvaluateSchedule(candidateRoutes{k},model,cache);
end
[~,order] = sort(costs,'ascend');
keep = order(1:min(nPop,numel(order)));
initialPositions = zeros(nPop,numel(activeIDs));
for k = 1:numel(keep)
    initialPositions(k,:) = RouteToPosition(candidateRoutes{keep(k)},activeIDs);
end
for k = numel(keep)+1:nPop
    initialPositions(k,:) = rand(1,numel(activeIDs));
end

info.baseRoute = baseRoute;
info.missingIDs = missingIDs;
info.candidateCount = numel(candidateRoutes);
info.keptCount = numel(keep);
info.evaluations = numel(candidateRoutes);
end

function route = InsertAt(route,id,position)
route = [route(1:position-1),id,route(position:end)];
end

function position = RouteToPosition(route,activeIDs)
position = zeros(1,numel(activeIDs));
for k = 1:numel(route)
    index = find(activeIDs==route(k),1);
    position(index) = (k-1)/max(numel(route),1)+0.001*rand;
end
position = min(max(position,0),1);
end

function key = RouteKey(route)
if isempty(route)
    key = '';
else
    key = sprintf('%d_',route);
end
end



