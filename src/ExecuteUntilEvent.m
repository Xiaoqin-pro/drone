function state = ExecuteUntilEvent(solution,model,eventTime)
%EXECUTEUNTILEEVENT 沿已规划航段执行，支持事件发生在航段中间
routeIDs = solution.Detail.routeIDs;
routeLegs = solution.Detail.routeLegs;
currentTime = model.startTime;
completedIDs = zeros(0,1);

for k = 1:numel(routeIDs)
    customerID = routeIDs(k);
    leg = routeLegs{k};
    points = leg.points;
    for s = 1:size(points,1)-1
        from = points(s,:);
        to = points(s+1,:);
        distance = norm(to-from);
        travelTime = distance/model.speed;
        if eventTime<=currentTime+travelTime
            ratio = max(0,min(1,(eventTime-currentTime)/max(travelTime,eps)));
            state.position = from+ratio*(to-from);
            state.time = eventTime;
            state.completedIDs = completedIDs;
            state.currentLeg = [from;to];
            state.inServiceCustomer = 0;
            return;
        end
        currentTime = currentTime+travelTime;
    end

    localCustomer = find(model.customerIDs==customerID,1);
    serviceStart = max(currentTime,model.windows(localCustomer,1));
    if eventTime<=serviceStart
        state.position = points(end,:);
        state.time = eventTime;
        state.completedIDs = completedIDs;
        state.currentLeg = [];
        state.inServiceCustomer = customerID;
        return;
    end
    serviceEnd = serviceStart+model.service(localCustomer);
    if eventTime<=serviceEnd
        state.position = points(end,:);
        state.time = eventTime;
        state.completedIDs = completedIDs;
        state.currentLeg = [];
        state.inServiceCustomer = customerID;
        return;
    end
    currentTime = serviceEnd;
    completedIDs(end+1,1) = customerID; %#ok<AGROW>
end

state.position = model.homePosition;
state.time = eventTime;
state.completedIDs = completedIDs;
state.currentLeg = [];
state.inServiceCustomer = 0;
end
