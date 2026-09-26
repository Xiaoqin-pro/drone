function [cost,detail] = EvaluateSchedule(routeIDs,model,cache)
%EVALUATESCHEDULE 使用航段缓存评价当前活动客户访问顺序
routeIDs = routeIDs(:)';
currentTime = model.startTime;
totalDistance = 0;
totalLate = 0;
totalWaiting = 0;
totalTerrainViolation = 0;
totalObstacleViolation = 0;
minClearance = inf;
smoothness = 0;
records = zeros(numel(routeIDs),9);
routeLegs = cell(numel(routeIDs)+1,1);
pathPoints = [];

previousNode = cache.startIndex;
for k = 1:numel(routeIDs)+1
    if k<=numel(routeIDs)
        customerID = routeIDs(k);
        targetNode = find(cache.customerIDs==customerID,1)+1;
    else
        customerID = 0;
        targetNode = cache.homeIndex;
    end

    leg = cache.legs{previousNode,targetNode};
    routeLegs{k} = leg;
    totalDistance = totalDistance+leg.distance;
    totalTerrainViolation = totalTerrainViolation+leg.terrainViolation;
    totalObstacleViolation = totalObstacleViolation+leg.obstacleViolation;
    minClearance = min(minClearance,leg.minClearance);
    smoothness = smoothness+leg.smoothness;

    if isempty(pathPoints)
        pathPoints = leg.points;
    else
        pathPoints = [pathPoints;leg.points(2:end,:)]; %#ok<AGROW>
    end

    if k<=numel(routeIDs)
        localCustomer = find(model.customerIDs==customerID,1);
        arrival = currentTime+leg.distance/model.speed;
        serviceStart = max(arrival,model.windows(localCustomer,1));
        late = max(0,serviceStart-model.windows(localCustomer,2));
        waiting = max(0,serviceStart-arrival);
        totalLate = totalLate+late;
        totalWaiting = totalWaiting+waiting;
        records(k,:) = [customerID,arrival,serviceStart,late,waiting, ...
            leg.distance,leg.minClearance,leg.obstacleViolation,totalDistance];
        currentTime = serviceStart+model.service(localCustomer);
    end
    previousNode = targetNode;
end

cost = totalDistance ...
    + model.latePenalty*totalLate ...
    + model.terrainPenalty*totalTerrainViolation ...
    + model.obstaclePenalty*totalObstacleViolation ...
    + model.smoothPenalty*smoothness;

detail.cost = cost;
detail.routeIDs = routeIDs;
detail.pathPoints = pathPoints;
detail.routeLegs = routeLegs;
detail.distance = totalDistance;
detail.totalLate = totalLate;
detail.totalWaiting = totalWaiting;
detail.totalTerrainViolation = totalTerrainViolation;
detail.totalObstacleViolation = totalObstacleViolation;
detail.totalViolation = totalTerrainViolation+totalObstacleViolation;
detail.minClearance = minClearance;
detail.smoothness = smoothness;
detail.records = records;
detail.finishTime = currentTime;
detail.isSafetyFeasible = detail.totalViolation<=1e-9;
detail.isTimeFeasible = detail.totalLate<=1e-9;
detail.isFeasible = detail.isSafetyFeasible && detail.isTimeFeasible;
end

