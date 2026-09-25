function report = VerifyScenarioFeasibility(model,cache,activeIDs,nSamples)
%VERIFYSCENARIOFEASIBILITY 生成参考可行解候选，避免把不可行实例误判为算法失败
if nargin<4
    nSamples = 2000;
end
activeIDs = activeIDs(:)';
bestCost = inf;
bestDetail = [];
bestRoute = [];
firstFeasibleFE = inf;

candidates = {activeIDs,NearestNeighborRoute(activeIDs,model)};
for k = 1:numel(candidates)
    [cost,detail] = EvaluateSchedule(candidates{k},model,cache);
    if cost<bestCost
        bestCost = cost; bestDetail = detail; bestRoute = candidates{k};
    end
    if detail.isFeasible && isinf(firstFeasibleFE)
        firstFeasibleFE = k;
    end
end

rng(9090,'twister');
for k = 1:nSamples
    route = activeIDs(randperm(numel(activeIDs)));
    [cost,detail] = EvaluateSchedule(route,model,cache);
    if cost<bestCost
        bestCost = cost; bestDetail = detail; bestRoute = route;
    end
    if detail.isFeasible && isinf(firstFeasibleFE)
        firstFeasibleFE = k+numel(candidates);
        break;
    end
end

report.isFound = ~isinf(firstFeasibleFE);
report.firstFeasibleFE = firstFeasibleFE;
report.referenceCost = bestCost;
report.referenceRoute = bestRoute;
report.referenceDetail = bestDetail;
report.sampleCount = nSamples;
end

function route = NearestNeighborRoute(activeIDs,model)
remaining = activeIDs(:)';
route = zeros(1,numel(remaining));
current = model.depot;
for k = 1:numel(route)
    coordinates = model.customerXYZ(remaining,:);
    distance = sqrt(sum((coordinates-current).^2,2));
    [~,index] = min(distance);
    route(k) = remaining(index);
    current = coordinates(index,:);
    remaining(index) = [];
end
end
