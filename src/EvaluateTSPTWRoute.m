function [cost,detail] = EvaluateTSPTWRoute(route,instance,options)
%EVALUATETSPTWROUTE 评价单仓库TSPTW访问顺序
%   route只包含客户节点编号，不包含仓库。路线默认从1出发并回到1。

if nargin<3 || isempty(options)
    options = struct();
end
if ~isfield(options,'latePenalty') || isempty(options.latePenalty)
    options.latePenalty = 1000;
end
if ~isfield(options,'waitPenalty') || isempty(options.waitPenalty)
    options.waitPenalty = 0;
end

route = route(:)';
expected = instance.customerIDs(:)';
if numel(unique(route))~=numel(route) || any(~ismember(route,expected)) ...
        || numel(route)~=numel(expected)
    error('route必须恰好包含每个客户节点一次。');
end

currentTime = instance.windows(instance.depotIndex,1);
totalDistance = 0;
totalLate = 0;
totalWaiting = 0;
records = zeros(numel(route),7);
previousNode = instance.depotIndex;

for k = 1:numel(route)
    node = route(k);
    travel = instance.costMatrix(previousNode,node);
    arrival = currentTime+travel;
    serviceStart = max(arrival,instance.windows(node,1));
    waiting = max(0,serviceStart-arrival);
    late = max(0,serviceStart-instance.windows(node,2));
    totalDistance = totalDistance+travel;
    totalWaiting = totalWaiting+waiting;
    totalLate = totalLate+late;
    records(k,:) = [node,arrival,serviceStart,late,waiting,travel,totalDistance];
    currentTime = serviceStart+instance.service(node);
    previousNode = node;
end

returnTravel = instance.costMatrix(previousNode,instance.depotIndex);
totalDistance = totalDistance+returnTravel;
finishTime = currentTime+returnTravel;
latePenalty = options.latePenalty*totalLate;
waitPenalty = options.waitPenalty*totalWaiting;
cost = totalDistance+latePenalty+waitPenalty;

detail.cost = cost;
detail.route = route;
detail.distance = totalDistance;
detail.totalLate = totalLate;
detail.totalWaiting = totalWaiting;
detail.finishTime = finishTime;
detail.records = records;
detail.isTimeFeasible = totalLate<=1e-9;
detail.isFeasible = detail.isTimeFeasible;
detail.latePenalty = latePenalty;
detail.waitPenalty = waitPenalty;
end

